#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_description/se_tools.h>
#include <mantis_router_base/mantis_router_base.h>
#include <mantis_router_base/ControlParamsConfig.h>
#include <mantis_router_base/BaseMovementAction.h>

#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include <vector>

MantisRouterBase::MantisRouterBase() :
	nh_(),
	nhp_("~"),
	p_(nh_),
	s_(nh_, p_),
	solver_(p_, s_),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_path_rate_(50.0),
	spline_start_(0),
	spline_duration_(0),
	spline_in_progress_(false),
	use_dirty_derivative_(false),
	as_(nhp_, "action", false),
	dyncfg_control_settings_(ros::NodeHandle(nhp_, "control_settings")) {

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("path_rate", param_path_rate_, param_path_rate_);

	dyncfg_control_settings_.setCallback(boost::bind(&MantisRouterBase::callback_cfg_control_settings, this, _1, _2));

	vbe_last_ = Eigen::Matrix<double,6,1>::Zero();

	if( p_.wait_for_params() && s_.wait_for_state() ) {
		pub_tri_ = nhp_.advertise<mavros_msgs::PositionTarget>("output/triplet", 10);
		pub_pose_base_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/base", 10);
		pub_pose_track_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/track", 10);

		timer_path_ = nhp_.createTimer(ros::Duration(1.0/param_path_rate_), &MantisRouterBase::callback_path, this );

		as_.start();

		ROS_INFO_ONCE("Augmented path control loaded!");
	} else {
		ROS_WARN("Augmented path control shutting down.");
		ros::shutdown();
	}
}

MantisRouterBase::~MantisRouterBase() {
}

void MantisRouterBase::callback_cfg_control_settings(mantis_router_base::ControlParamsConfig &config, uint32_t level) {
	param_tracked_frame_ = config.tracked_frame;
	param_manipulator_jacobian_a_ = config.manipulator_jacobian_alpha;
	param_send_reference_feedback_ = config.reference_feedback;
}

Eigen::Affine3d MantisRouterBase::calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe) {
	//Use this yaw only rotation to set the direction of the base (and thus end effector)
	Eigen::Matrix3d br_sp = MDTools::extract_yaw_matrix(ge_sp.linear());

	//Construct the true end effector
	Eigen::Affine3d sp = Eigen::Affine3d::Identity();
	sp.linear() = br_sp*gbe.linear();
	sp.translation() = ge_sp.translation();

	//Use the inverse transform to get the true base transform
	return sp*gbe.inverse();
}

Eigen::Vector3d MantisRouterBase::calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::VectorXd &vbe) {
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

void MantisRouterBase::set_action_goal( void ) {
	mantis_router_base::BaseMovementGoal goal = *(as_.acceptNewGoal());

	if( (goal.duration > ros::Duration(0) ) &&
		(goal.positions.size() >= 2) &&
		(goal.yaws.size() >= 2) ) {

		spline_in_progress_ = true;

		spline_start_ = ( goal.start == ros::Time(0) ) ? ros::Time::now() : goal.start;
		spline_duration_ = goal.duration;

		std::vector<double>positions_x;
		std::vector<double>positions_y;
		std::vector<double>positions_z;

		for(int i=0; i<goal.positions.size(); i++) {
			positions_x.push_back(goal.positions[i].x);
			positions_y.push_back(goal.positions[i].y);
			positions_z.push_back(goal.positions[i].z);
		}
		ROS_INFO("INTERP START");
		spline_x_ = tinyspline::Utils::interpolateCubic(&positions_x, 1);
		spline_y_ = tinyspline::Utils::interpolateCubic(&positions_y, 1);
		spline_z_ = tinyspline::Utils::interpolateCubic(&positions_z, 1);
		spline_r_ = tinyspline::Utils::interpolateCubic(&goal.yaws, 1);
		ROS_INFO("INTERP END");

		//Smooth out control points to give a nicer fit
		std::vector<tinyspline::real> ctrlp_x = spline_x_.controlPoints();
		std::vector<tinyspline::real> ctrlp_y = spline_y_.controlPoints();
		std::vector<tinyspline::real> ctrlp_z = spline_z_.controlPoints();
		std::vector<tinyspline::real> ctrlp_r = spline_r_.controlPoints();
		ROS_ASSERT_MSG( ctrlp_x.size() >= 4, "Number of pos_x control points is <4 (%i)", (int)ctrlp_x.size());
		ROS_ASSERT_MSG( ctrlp_y.size() >= 4, "Number of pos_y control points is <4 (%i)", (int)ctrlp_y.size());
		ROS_ASSERT_MSG( ctrlp_z.size() >= 4, "Number of pos_z control points is <4 (%i)", (int)ctrlp_z.size());
		ROS_ASSERT_MSG( ctrlp_r.size() >= 4, "Number of yaw control points is <4 (%i)", (int)ctrlp_r.size());

		ctrlp_x.at(1) = ctrlp_x.front();
		ctrlp_x.at(ctrlp_x.size() - 2) = ctrlp_x.back();
		spline_x_.setControlPoints(ctrlp_x);
		ctrlp_y.at(1) = ctrlp_y.front();
		ctrlp_y.at(ctrlp_y.size() - 2) = ctrlp_y.back();
		spline_y_.setControlPoints(ctrlp_y);
		ctrlp_z.at(1) = ctrlp_z.front();
		ctrlp_z.at(ctrlp_z.size() - 2) = ctrlp_z.back();
		spline_z_.setControlPoints(ctrlp_z);
		ctrlp_r.at(1) = ctrlp_r.front();
		ctrlp_r.at(ctrlp_r.size() - 2) = ctrlp_r.back();
		spline_r_.setControlPoints(ctrlp_r);

		try {
			spline_xd_ = spline_x_.derive();
			spline_yd_ = spline_y_.derive();
			spline_zd_ = spline_z_.derive();
			spline_rd_ = spline_r_.derive();

			use_dirty_derivative_ = false;
		}
		catch(std::runtime_error) {
			use_dirty_derivative_ = true;
		}

		spline_pos_start_ = MDTools::vector_from_msg(goal.positions.front());
		spline_pos_end_ = MDTools::vector_from_msg(goal.positions.back());
		spline_rot_start_ = goal.yaws.front();
		spline_rot_end_ = goal.yaws.back();

		ROS_INFO( "Router Base: creating position spline connecting %i points", (int)goal.positions.size() );
		ROS_INFO( "Router Base: creating rotation spline connecting %i points", (int)goal.yaws.size() );
	} else {
		spline_in_progress_ = false;
		as_.setAborted();

		ROS_ERROR( "Router Base: at least 2 positions/yaws must be specified (%i/%i), and duration must be >0 (%0.4f)", (int)goal.positions.size(), (int)goal.yaws.size(), goal.duration.toSec() );
	}
}

void MantisRouterBase::get_spline_reference(tinyspline::BSpline& spline,
											tinyspline::BSpline& splined,
											double& pos,
											double& vel,
											const double u) {
	// x values need to be scaled down in extraction as well.
	ROS_ASSERT_MSG((u >= 0.0) && (u <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");

	std::vector<tinyspline::real> vp = spline(u).result();
	pos = vp[0];

	if(!use_dirty_derivative_) {
		std::vector<tinyspline::real> vv = splined(u).result();
		vel = vv[0];
	} else {
		//XXX: Manually derrive over a short period as proper derivative can't be calculated using this library
		double dt = 0.02;
		//Shorten time to ensure that 0.0<=u<=1.0 is preserved
		double ul = u - dt;
		double uh = u + dt;
		ul = (ul >= 0.0) ? ul : 0.0;
		uh = (uh <= 1.0) ? uh : 1.0;

		std::vector<tinyspline::real> vdl = spline(ul).result();
		std::vector<tinyspline::real> vdh = spline(uh).result();

		vel = (vdh[0] - vdl[0]) / (2*dt);
	}
}

bool MantisRouterBase::get_reference(Eigen::Vector3d &pos,
									 Eigen::Vector3d &vel,
									 double &rpos,
									 double &rrate,
									 const ros::Time tc) {
	bool success = false;

	//If a valid input has been received
	if( spline_start_ > ros::Time(0) ) {
		//If in progress, calculate the lastest reference
		if( spline_in_progress_ ) {
			if( tc < spline_start_ ) {
				//Have no begun, stay at start position
				pos = spline_pos_start_;
				rpos = spline_rot_start_;
				vel = Eigen::Vector3d::Zero();
				rrate = 0.0;

				mantis_router_base::BaseMovementFeedback feedback;
				feedback.progress = -1.0;
				feedback.position = MDTools::vector_from_eig(pos);
				feedback.velocity = MDTools::vector_from_eig(vel);
				feedback.yaw = rpos;
				feedback.yawrate = rrate;

				as_.publishFeedback(feedback);
			} else if( tc <= (spline_start_ + spline_duration_) ) {
				double t_norm = normalize((tc - spline_start_).toSec(), 0.0, spline_duration_.toSec());
				double npx = 0.0;
				double npy = 0.0;
				double npz = 0.0;
				double npr = 0.0;
				double nvx = 0.0;
				double nvy = 0.0;
				double nvz = 0.0;
				double nvr = 0.0;

				get_spline_reference(spline_x_, spline_xd_, npx, nvx, t_norm);
				get_spline_reference(spline_y_, spline_yd_, npy, nvy, t_norm);
				get_spline_reference(spline_z_, spline_zd_, npz, nvz, t_norm);
				get_spline_reference(spline_r_, spline_rd_, npr, nvr, t_norm);

				pos = Eigen::Vector3d(npx,npy,npz);
				vel = Eigen::Vector3d(nvx,nvy,nvz) / spline_duration_.toSec();
				rpos = npr;
				rrate = nvr / spline_duration_.toSec();

				mantis_router_base::BaseMovementFeedback feedback;
				feedback.progress = t_norm;
				feedback.position = MDTools::vector_from_eig(pos);
				feedback.velocity = MDTools::vector_from_eig(vel);
				feedback.yaw = rpos;
				feedback.yawrate = rrate;

				as_.publishFeedback(feedback);
			} else {
				//We just finished, so send a result
				pos = spline_pos_end_;
				rpos = spline_rot_end_;
				vel = Eigen::Vector3d::Zero();
				rrate = 0.0;

				mantis_router_base::BaseMovementResult result;
				result.position_final = MDTools::vector_from_eig(pos);
				result.yaw_final = rpos;
				as_.setSucceeded(result);

				spline_in_progress_ = false;
				ROS_INFO( "Router Base: action finished" );
			}
		} else {
			pos = spline_pos_end_;
			rpos = spline_rot_end_;
			vel = Eigen::Vector3d::Zero();
			rrate = 0.0;
		}

		success = true;
	}

	return success;
}

void MantisRouterBase::callback_path(const ros::TimerEvent& e) {
	double dt = (e.current_real - e.last_real).toSec();
	bool success = false;

	//Check for a new goal
	if( as_.isNewGoalAvailable() ) {
		set_action_goal();
	}

	// Check that preempt has not been requested by the client
	if( as_.isPreemptRequested() ) {
		ROS_INFO("Router Base: Preempted");
		as_.setPreempted();
		spline_in_progress_ = false;
	}

	if( ( p_.ok() ) && ( s_.ok() ) &&
		( spline_start_ > ros::Time(0) ) ) {

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
			track_alter_success = solver_.calculate_gbe( gbe );
			ksolver_success = track_alter_success;
		} else if(param_tracked_frame_ > 0) {
			//Compute transform for all joints in the chain to get base to specified joint
			track_alter_success = solver_.calculate_gxy( gbe, 0, param_tracked_frame_ );
			ksolver_success = track_alter_success;
		}

		Eigen::Vector3d ref_pos;
		Eigen::Vector3d ref_vel;
		double ref_rpos;
		double ref_rvel;

		//Make certain we have a valid reference
		if(ksolver_success && get_reference(ref_pos, ref_vel, ref_rpos, ref_rvel, e.current_real) ) {
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
						calc_manip_velocity = solver_.calculate_vbe( vbe );

					if(param_tracked_frame_ > 0)
						calc_manip_velocity = solver_.calculate_vbx( vbe, param_tracked_frame_ );

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
				vbe_last_ = Eigen::Matrix<double,6,1>::Zero();
			}

			Eigen::Vector3d gv_sp_b = g_sp.linear()*gv_sp;

			success = true;

			mavros_msgs::PositionTarget msg_tri_out;
			msg_tri_out.header.stamp = e.current_real;
			msg_tri_out.header.frame_id = param_frame_id_;
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
				msg_base_out.header.frame_id = param_frame_id_;
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
