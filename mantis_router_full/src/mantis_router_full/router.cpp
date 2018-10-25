#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_description/se_tools.h>
#include <mantis_router_full/router.h>
#include <mantis_router_full/joint.h>
#include <mantis_router_full/ControlParamsConfig.h>

#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include <vector>

namespace MantisRouterFull {

Router::Router() :
	nh_(),
	nhp_("~"),
	p_(nh_),
	s_(nh_, p_),
	solver_(p_, s_),
	contrail_(nhp_),
	param_update_rate_(50.0),
	dyncfg_control_settings_(ros::NodeHandle(nhp_, "control_settings")) {

	nhp_.param("update_rate", param_update_rate_, param_update_rate_);

	dyncfg_control_settings_.setCallback(boost::bind(&MantisRouterFull::Router::callback_cfg_control_settings, this, _1, _2));

	vbe_last_ = Eigen::Matrix<double,6,1>::Zero();

	if( p_.wait_for_params() && s_.wait_for_state() ) {
		pub_tri_ = nhp_.advertise<mavros_msgs::PositionTarget>("output/triplet", 10);
		pub_pose_base_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/base", 10);
		pub_pose_track_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/track", 10);

		configure_joint_routers();

		timer_ = nhp_.createTimer(ros::Duration(1.0/param_update_rate_), &MantisRouterFull::Router::callback_timer, this );

		ROS_INFO_ONCE("Augmented router control loaded!");
	} else {
		ROS_WARN("Augmented router control shutting down.");
		ros::shutdown();
	}
}

Router::~Router() {
	remove_joint_routers();
}

void Router::callback_cfg_control_settings(mantis_router_full::ControlParamsConfig &config, uint32_t level) {
	param_tracked_frame_ = config.tracked_frame;
	param_manipulator_jacobian_a_ = config.manipulator_jacobian_alpha;
	param_send_reference_feedback_ = config.reference_feedback;
}

void Router::configure_joint_routers( void ) {
	remove_joint_routers();

	joint_routers_.resize( p_.get_dynamic_joint_num() );
	spawn_stamp_ = p_.time_configuration_change();

	int cc = 0;
	for(int i = 0; i < p_.get_joint_num(); i++) {
		if( p_.joint(i).type != "static" ) {
			joint_routers_[cc] = new MantisRouterFull::Joint(nhp_, p_.joint(i).name);
			cc++;
		}
	}

	ROS_INFO_STREAM("Spawned " << joint_routers_.size() << " joint routers");
}

void Router::remove_joint_routers( void ) {
	for(int i = 0; i < joint_routers_.size(); i++) {
		if(joint_routers_[i] != NULL) {
			delete joint_routers_[i];
			joint_routers_[i] = NULL;
		}
	}

	joint_routers_.clear();
}

Eigen::Affine3d Router::calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe) {
	//Use this yaw only rotation to set the direction of the base (and thus end effector)
	Eigen::Matrix3d br_sp = MDTools::extract_yaw_matrix(ge_sp.linear());

	//Construct the true end effector
	Eigen::Affine3d sp = Eigen::Affine3d::Identity();
	sp.linear() = br_sp*gbe.linear();
	sp.translation() = ge_sp.translation();

	//Use the inverse transform to get the true base transform
	return sp*gbe.inverse();
}

Eigen::Vector3d Router::calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::VectorXd &vbe) {
	//Velocity of the end effector in the end effector frame
	//Eigen::VectorXd vbe = Je*rd;

	//Eigen::Affine3d Vbe;
	//Vbe.translation() << vbe.segment(0,3);
	//Vbe.linear() << vee_up(vbe.segment(3,3));

	//Get linear velocity in the world frame
	Eigen::Vector3d Ve = Re*vbe.segment(0,3);

	//Only care about the translation movements
	return gev_sp - Ve;
}

void Router::callback_timer(const ros::TimerEvent& e) {
	double dt = (e.current_real - e.last_real).toSec();
	bool success = false;

	if(p_.time_configuration_change() != spawn_stamp_) {
		ROS_WARN("Reconfiguring router configuration");
		configure_joint_routers();
	}

	if( ( p_.ok() ) && ( s_.ok() ) && ( contrail_.has_reference(e.current_real) ) ) {
		//For all joint routers
		ROS_ASSERT_MSG(joint_routers_.size() == p_.get_dynamic_joint_num(), "Joint router and dynamic joint param inconsistent (%i!=%i)", (int)joint_routers_.size(), (int)p_.get_dynamic_joint_num());

		Eigen::VectorXd ref_r = Eigen::VectorXd::Zero( p_.get_dynamic_joint_num() );
		Eigen::VectorXd ref_rd = Eigen::VectorXd::Zero( p_.get_dynamic_joint_num() );

		//Update all the joint routers
		for(int i = 0; i < p_.get_dynamic_joint_num(); i++) {
			joint_routers_[i]->update( e.current_real );

			//If possible, prefer to use the joint reference
			//over the joint state, as this will potentially
			//avoid any feedback oscillations in the alterations
			if( joint_routers_[i]->has_reference() ) {
				ref_r[i] = joint_routers_[i]->get_r();
				ref_rd[i] = joint_routers_[i]->get_rd();
			} else {
				ref_r[i] = s_.r()[i];
				ref_rd[i] = s_.rd()[i];
			}
		}

		solver_.load_ref(ref_r, ref_rd);

		//Handle the rest of the base routing
		Eigen::Affine3d g_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gv_sp = Eigen::Vector3d::Zero();
		Eigen::Affine3d ge_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gev_sp = Eigen::Vector3d::Zero();
		Eigen::Affine3d gbe = Eigen::Affine3d::Identity();
		bool ksolver_success = false;
		bool track_alter_success = false;

		if(param_tracked_frame_ == 0) {
			//Just track the base
			ksolver_success = true;
		} else if(param_tracked_frame_ == -1) {
			//Compute transform for all joints in the chain to get base to end effector
			track_alter_success = solver_.calculate_gbe_ref( gbe );
			ksolver_success = track_alter_success;
		} else if(param_tracked_frame_ > 0) {
			//Compute transform for all joints in the chain to get base to specified joint
			track_alter_success = solver_.calculate_gxy_ref( gbe, 0, param_tracked_frame_ );
			ksolver_success = track_alter_success;
		}

		Eigen::Vector3d ref_pos;
		Eigen::Vector3d ref_vel;
		double ref_rpos;
		double ref_rvel;

		//Make certain we have a valid reference
		if(ksolver_success && contrail_.get_reference(ref_pos, ref_vel, ref_rpos, ref_rvel, e.current_real ) ) {
			//Fill in the current goals
			ge_sp.translation() = ref_pos;
			ge_sp.linear() = Eigen::AngleAxisd(ref_rpos, Eigen::Vector3d::UnitZ()).toRotationMatrix();
			gev_sp = ref_vel;

			if( track_alter_success ) {
				//Calculate the tracking offset for the base
				g_sp = calc_goal_base_transform(ge_sp, gbe);

				//Compensate for velocity terms in manipulator movement
				//This is more accurate, but can make things more unstable
				if(param_manipulator_jacobian_a_ > 0.0) {
					bool calc_manip_velocity = false;
					Eigen::VectorXd vbe;

					if(param_tracked_frame_ == -1)
						calc_manip_velocity = solver_.calculate_vbe_ref( vbe );

					if(param_tracked_frame_ > 0)
						calc_manip_velocity = solver_.calculate_vbx_ref( vbe, param_tracked_frame_ );

					if( calc_manip_velocity ) {
						vbe_last_ =((1.0 - param_manipulator_jacobian_a_) * vbe_last_) + (param_manipulator_jacobian_a_ * vbe);

						//Manipulator velocity in the world frame
						Eigen::Matrix3d br_sp = MDTools::extract_yaw_matrix(g_sp.linear());
						Eigen::Vector3d Ve = br_sp*vbe_last_.segment(0,3);
						//Subtract from setpoint to compensate base movements
						gv_sp = gev_sp - Ve;
					} else {
						ROS_ERROR_THROTTLE(2.0, "Unable to use manipulator Jacobian");
						gv_sp = gev_sp;
						vbe_last_ = Eigen::Matrix<double,6,1>::Zero();
					}
				} else {
					gv_sp = gev_sp;
					vbe_last_ = Eigen::Matrix<double,6,1>::Zero();
				}
			} else {
				//Just track the robot base
				g_sp = ge_sp;
				gv_sp = gev_sp;
				gbe = Eigen::Affine3d::Identity();
				vbe_last_ = Eigen::Matrix<double,6,1>::Zero();
			}

			Eigen::Vector3d gv_sp_b = g_sp.linear()*gv_sp;

			success = true;

			contrail_.check_end_reached(s_.g()*gbe);

			mavros_msgs::PositionTarget msg_tri_out;
			msg_tri_out.header.stamp = e.current_real;
			msg_tri_out.header.frame_id = s_.frame_id();
			msg_tri_out.coordinate_frame = msg_tri_out.FRAME_LOCAL_NED;
			msg_tri_out.type_mask = msg_tri_out.IGNORE_AFX | msg_tri_out.IGNORE_AFY | msg_tri_out.IGNORE_AFZ | msg_tri_out.FORCE;
			msg_tri_out.position = MDTools::point_from_eig(g_sp.translation());
			msg_tri_out.velocity = MDTools::vector_from_eig(gv_sp);
			//Eigen::Vector3d ea = g_sp.linear().eulerAngles(2,1,0);
			msg_tri_out.yaw = MDTools::extract_yaw(Eigen::Quaterniond(g_sp.linear()));
			msg_tri_out.yaw_rate = ref_rvel;

			pub_tri_.publish(msg_tri_out);

			if(param_send_reference_feedback_) {
				geometry_msgs::PoseStamped msg_base_out;
				geometry_msgs::PoseStamped msg_track_out;
				msg_base_out.header.stamp = e.current_real;
				msg_base_out.header.frame_id = s_.frame_id();
				msg_track_out.header = msg_base_out.header;

				msg_base_out.pose = MDTools::pose_from_eig(g_sp);
				msg_track_out.pose = MDTools::pose_from_eig(ge_sp);

				pub_pose_base_.publish(msg_base_out);
				pub_pose_track_.publish(msg_track_out);
			}
		}
	}

	//Reset some values if we had an failed calc
	if(!success) {
		vbe_last_ = Eigen::Matrix<double,6,1>::Zero();
	}
}

}
