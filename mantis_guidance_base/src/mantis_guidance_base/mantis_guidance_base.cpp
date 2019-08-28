#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <mantis_description/se_tools.h>
#include <mantis_guidance_base/ControlParamsConfig.h>
#include <mantis_guidance_base/mantis_guidance_base.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <vector>

MantisGuidanceBase::MantisGuidanceBase()
	: nh_()
	, nhp_( "~" )
	, p_( nh_ )
	, s_( nh_, p_ )
	, solver_( p_, s_ )
	, param_frame_id_( "map" )
	, contrail_( nhp_, "map", true )
	, param_model_id_( "mantis_uav" )
	, param_path_rate_( 50.0 )
	, dyncfg_control_settings_( ros::NodeHandle( nhp_, "control_settings" ) ) {

	nhp_.param( "frame_id", param_frame_id_, param_frame_id_ );
	nhp_.param( "model_id", param_model_id_, param_model_id_ );
	nhp_.param( "path_rate", param_path_rate_, param_path_rate_ );

	dyncfg_control_settings_.setCallback( boost::bind(
		&MantisGuidanceBase::callback_cfg_control_settings, this, _1, _2 ) );

	vbe_last_ = Eigen::Matrix<double, 6, 1>::Zero();

	if ( p_.wait_for_params() && s_.wait_for_state() ) {
		pub_tri_ = nhp_.advertise<mavros_msgs::PositionTarget>( "output/triplet", 10 );
		pub_pose_base_ = nhp_.advertise<geometry_msgs::PoseStamped>( "feedback/base", 10 );
		pub_pose_track_ = nhp_.advertise<geometry_msgs::PoseStamped>( "feedback/track", 10 );

		timer_ = nhp_.createTimer( ros::Duration( 1.0 / param_path_rate_ ),
			&MantisGuidanceBase::callback_timer, this );

		ROS_INFO_ONCE( "Multirotor base guidance loaded!" );
	} else {
		ROS_WARN( "Multirotor base guidance shutting down." );
		ros::shutdown();
	}
}

MantisGuidanceBase::~MantisGuidanceBase() {
}

void MantisGuidanceBase::callback_cfg_control_settings(
	mantis_guidance_base::ControlParamsConfig& config, uint32_t level ) {
	param_tracked_frame_ = config.tracked_frame;
	param_manipulator_jacobian_a_ = config.manipulator_jacobian_alpha;
	param_send_reference_feedback_ = config.reference_feedback;
}

Eigen::Affine3d
MantisGuidanceBase::calc_goal_base_transform( const Eigen::Affine3d& ge_sp,
	const Eigen::Affine3d& gbe ) {
	// Use this yaw only rotation to set the direction of the base (and thus end
	// effector)
	Eigen::Matrix3d br_sp = MDTools::extract_yaw_matrix( ge_sp.linear() );

	// Construct the true end effector
	Eigen::Affine3d sp = Eigen::Affine3d::Identity();
	sp.linear() = br_sp * gbe.linear();
	sp.translation() = ge_sp.translation();

	// Use the inverse transform to get the true base transform
	return sp * gbe.inverse();
}

Eigen::Vector3d
MantisGuidanceBase::calc_goal_base_velocity( const Eigen::Vector3d& gev_sp,
	const Eigen::Matrix3d& Re,
	const Eigen::VectorXd& vbe ) {
	// Velocity of the end effector in the end effector frame
	// Eigen::VectorXd vbe = Je*rd;

	// Eigen::Affine3d Vbe;
	// Vbe.translation() << vbe.segment(0,3);
	// Vbe.linear() << vee_up(vbe.segment(3,3));

	// Get linear velocity in the world frame
	Eigen::Vector3d Ve = Re * vbe.segment( 0, 3 );

	// Only care about the translation movements
	return gev_sp - Ve;
}

void MantisGuidanceBase::callback_timer( const ros::TimerEvent& e ) {
	double dt = ( e.current_real - e.last_real ).toSec();
	bool success = false;

	if ( ( p_.ok() ) && ( s_.ok() ) && ( contrail_.has_reference( e.current_real ) ) ) {

		Eigen::Affine3d g_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gv_sp = Eigen::Vector3d::Zero();
		Eigen::Affine3d ge_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gev_sp = Eigen::Vector3d::Zero();
		Eigen::Affine3d gbe = Eigen::Affine3d::Identity();
		bool ksolver_success = false;
		bool track_alter_success = false;

		if ( param_tracked_frame_ == 0 ) {
			// Just track the base
			ksolver_success = true;
		} else if ( param_tracked_frame_ == -1 ) {
			// Compute transform for all joints in the chain to get base to end
			// effector
			track_alter_success = solver_.calculate_gbe( gbe );
			ksolver_success = track_alter_success;
		} else if ( param_tracked_frame_ > 0 ) {
			// Compute transform for all joints in the chain to get base to specified
			// joint
			track_alter_success = solver_.calculate_gxy( gbe, 0, param_tracked_frame_ );
			ksolver_success = track_alter_success;
		}

		Eigen::Vector3d ref_pos;
		Eigen::Vector3d ref_vel;
		Eigen::Vector3d ref_acc;
		double ref_rpos;
		double ref_rvel;

		// Make certain we have a valid reference
		if ( ksolver_success && contrail_.get_reference( ref_pos, ref_vel, ref_acc, ref_rpos, ref_rvel, e.current_real, s_.g() ) ) {
			// Fill in the current goals
			ge_sp.translation() = ref_pos;
			ge_sp.linear() = Eigen::AngleAxisd( ref_rpos, Eigen::Vector3d::UnitZ() )
								 .toRotationMatrix();
			gev_sp = ref_vel;

			if ( track_alter_success ) {
				// Calculate the tracking offset for the base
				g_sp = calc_goal_base_transform( ge_sp, gbe );

				// Compensate for velocity terms in manipulator movement
				// This is more accurate, but can make things more unstable
				if ( param_manipulator_jacobian_a_ > 0.0 ) {
					bool calc_manip_velocity = false;
					Eigen::VectorXd vbe;

					if ( param_tracked_frame_ == -1 )
						calc_manip_velocity = solver_.calculate_vbe( vbe );

					if ( param_tracked_frame_ > 0 )
						calc_manip_velocity = solver_.calculate_vbx( vbe, param_tracked_frame_ );

					if ( calc_manip_velocity ) {
						vbe_last_ = ( ( 1.0 - param_manipulator_jacobian_a_ ) * vbe_last_ ) + ( param_manipulator_jacobian_a_ * vbe );

						// Manipulator velocity in the world frame
						Eigen::Matrix3d br_sp = MDTools::extract_yaw_matrix( g_sp.linear() );
						Eigen::Vector3d Ve = br_sp * vbe_last_.segment( 0, 3 );
						// Subtract from setpoint to compensate base movements
						gv_sp = gev_sp - Ve;
					} else {
						ROS_ERROR_THROTTLE( 2.0, "Unable to use manipulator Jacobian" );
						gv_sp = gev_sp;
						vbe_last_ = Eigen::Matrix<double, 6, 1>::Zero();
					}
				} else {
					gv_sp = gev_sp;
					vbe_last_ = Eigen::Matrix<double, 6, 1>::Zero();
				}
			} else {
				// Just track the robot base
				g_sp = ge_sp;
				gv_sp = gev_sp;
				gbe = Eigen::Affine3d::Identity();
				vbe_last_ = Eigen::Matrix<double, 6, 1>::Zero();
			}

			Eigen::Vector3d gv_sp_b = g_sp.linear() * gv_sp;

			success = true;

			contrail_.check_end_reached( s_.g() * gbe );

			mavros_msgs::PositionTarget msg_tri_out;
			msg_tri_out.header.stamp = e.current_real;
			msg_tri_out.header.frame_id = param_frame_id_;
			msg_tri_out.coordinate_frame = msg_tri_out.FRAME_LOCAL_NED;
			// msg_tri_out.type_mask = msg_tri_out.IGNORE_AFX | msg_tri_out.IGNORE_AFY
			// | msg_tri_out.IGNORE_AFZ | msg_tri_out.FORCE;
			msg_tri_out.position = MDTools::point_from_eig( g_sp.translation() );
			msg_tri_out.velocity = MDTools::vector_from_eig( gv_sp );
			msg_tri_out.acceleration_or_force = MDTools::vector_from_eig( ref_acc );
			// Eigen::Vector3d ea = g_sp.linear().eulerAngles(2,1,0);
			msg_tri_out.yaw = MDTools::extract_yaw( Eigen::Quaterniond( g_sp.linear() ) );
			msg_tri_out.yaw_rate = ref_rvel;

			pub_tri_.publish( msg_tri_out );

			if ( param_send_reference_feedback_ ) {
				geometry_msgs::PoseStamped msg_base_out;
				geometry_msgs::PoseStamped msg_track_out;
				msg_base_out.header.stamp = e.current_real;
				msg_base_out.header.frame_id = param_frame_id_;
				msg_track_out.header = msg_base_out.header;

				msg_base_out.pose = MDTools::pose_from_eig( g_sp );
				msg_track_out.pose = MDTools::pose_from_eig( ge_sp );

				pub_pose_base_.publish( msg_base_out );
				pub_pose_track_.publish( msg_track_out );
			}
		}
	}

	// Reset some values if we had an failed calc
	if ( !success ) {
		vbe_last_ = Eigen::Matrix<double, 6, 1>::Zero();
	}
}
