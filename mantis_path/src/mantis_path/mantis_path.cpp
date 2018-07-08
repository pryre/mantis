#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <contrail/path_extract.h>
#include <mantis_description/se_tools.h>
#include <mantis_path/mantis_path.h>
#include <mantis_path/ControlParamsConfig.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

MantisPath::MantisPath() :
	nh_(),
	nhp_("~"),
	p_(&nh_),
	s_(&nh_),
	solver_(&p_, &s_),
	ref_path_(&nhp_),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_path_rate_(50.0),
	dyncfg_control_settings_(ros::NodeHandle(nhp_, "control_settings")) {

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("path_rate", param_path_rate_, param_path_rate_);

	dyncfg_control_settings_.setCallback(boost::bind(&MantisPath::callback_cfg_control_settings, this, _1, _2));

	bool success = false;

	while(ros::ok() && (!success) ) {
		if(p_.ok() && s_.ok())
			success = true;

		ros::spinOnce();
		ros::Rate(5).sleep();
	}

	if(success) {
		pub_traj_ = nhp_.advertise<nav_msgs::Odometry>("output/traj", 10);
		pub_pose_base_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/base", 10);
		pub_pose_track_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/track", 10);

		timer_path_ = nhp_.createTimer(ros::Duration(1.0/param_path_rate_), &MantisPath::callback_path, this );

		ROS_INFO_ONCE("Augmented path control loaded!");
	} else {
		ROS_WARN("Augmented path control shutting down.");
		ros::shutdown();
	}
}

MantisPath::~MantisPath() {
}

void MantisPath::callback_cfg_control_settings(mantis_path::ControlParamsConfig &config, uint32_t level) {
	param_tracked_frame_ = config.tracked_frame;
	param_use_manipulator_jacobian_ = config.use_manipulator_jacobian;
	param_send_reference_feedback_ = config.reference_feedback;
}

void MantisPath::callback_path(const ros::TimerEvent& e) {
	double dt = (e.current_real - e.last_real).toSec();

	//Goal States

	if( ( ref_path_.has_valid_path() || ref_path_.has_valid_fallback() ) &&
		( p_.ok() ) && ( s_.ok() ) ) {

		Eigen::Affine3d g_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gv_sp = Eigen::Vector3d::Zero();
		Eigen::Affine3d ge_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gev_sp = Eigen::Vector3d::Zero();

		ref_path_.get_ref_state(ge_sp, gev_sp, e.current_real);


		Eigen::Affine3d gbe;
		bool track_alter_success = false;

		//Compute transform for all joints in the chain to get base to end effector
		if(param_tracked_frame_ == -1)
			track_alter_success = solver_.calculate_gbe( gbe );

		//Compute transform for all joints in the chain to get base to specified joint
		if(param_tracked_frame_ > 0)
			track_alter_success = solver_.calculate_gxy( gbe, 0, param_tracked_frame_ );

		if( track_alter_success ) {
			//Calculate the tracking offset for the base
			g_sp = calc_goal_base_transform(ge_sp, gbe);

			//Compensate for velocity terms in manipulator movement
			//This is more accurate, but can make things more unstable
			if(param_use_manipulator_jacobian_) {
				bool calc_manip_velocity = false;
				Eigen::VectorXd vbe;

				if(param_tracked_frame_ == -1)
					calc_manip_velocity = solver_.calculate_vbe( vbe );

				if(param_tracked_frame_ > 0)
					calc_manip_velocity = solver_.calculate_vbx( vbe, param_tracked_frame_ );

				if( calc_manip_velocity ) {
					//Manipulator velocity in the world frame
					Eigen::Vector3d Ve = s_.g().linear()*vbe.segment(0,3);
					//Subtract from setpoint to compensate base movements
					gv_sp = gev_sp - Ve;
				} else {
					ROS_ERROR_THROTTLE(2.0, "Unable to use manipulator Jacobian");
					gv_sp = gev_sp;
				}
			} else {
				gv_sp = gev_sp;
			}
		} else {
			//Just track the robot base
			g_sp = ge_sp;
			gv_sp = gev_sp;
		}

		Eigen::Vector3d gv_sp_b = g_sp.linear()*gv_sp;

		nav_msgs::Odometry msg_traj_out;
		msg_traj_out.header.stamp = e.current_real;
		msg_traj_out.header.frame_id = param_frame_id_;
		msg_traj_out.pose.pose = pose_from_eig(g_sp);
		msg_traj_out.twist.twist.linear = vector_from_eig(gv_sp_b);

		pub_traj_.publish(msg_traj_out);

		if(param_send_reference_feedback_) {
			geometry_msgs::PoseStamped msg_base_out;
			geometry_msgs::PoseStamped msg_track_out;
			msg_base_out.header.stamp = e.current_real;
			msg_base_out.header.frame_id = param_frame_id_;
			msg_track_out.header = msg_base_out.header;

			msg_base_out.pose = pose_from_eig(g_sp);
			msg_track_out.pose = pose_from_eig(ge_sp);

			pub_pose_base_.publish(msg_base_out);
			pub_pose_track_.publish(msg_track_out);
		}
	}
}
/*
Eigen::Vector3d MantisPath::position_from_msg(const geometry_msgs::Point p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond MantisPath::quaternion_from_msg(const geometry_msgs::Quaternion q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Affine3d MantisPath::affine_from_msg(const geometry_msgs::Pose pose) {
		Eigen::Affine3d a;

		a.translation() << position_from_msg(pose.position);
		a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

		return a;
}
*/
/*
geometry_msgs::Vector3 MantisPath::vector_from_eig(const Eigen::Vector3d &v) {
	geometry_msgs::Vector3 vec;

	vec.x = v.x();
	vec.y = v.y();
	vec.z = v.z();

	return vec;
}

geometry_msgs::Point MantisPath::point_from_eig(const Eigen::Vector3d &p) {
	geometry_msgs::Point point;

	point.x = p.x();
	point.y = p.y();
	point.z = p.z();

	return point;
}

geometry_msgs::Quaternion MantisPath::quaternion_from_eig(const Eigen::Quaterniond &q) {
	geometry_msgs::Quaternion quat;
	Eigen::Quaterniond qn = q.normalized();

	quat.w = qn.w();
	quat.x = qn.x();
	quat.y = qn.y();
	quat.z = qn.z();

	return quat;
}

geometry_msgs::Pose MantisPath::pose_from_eig(const Eigen::Affine3d &g) {
	geometry_msgs::Pose pose;

	pose.position = point_from_eig(g.translation());
	pose.orientation = quaternion_from_eig(Eigen::Quaterniond(g.linear()));

	return pose;
}
*/
Eigen::Affine3d MantisPath::calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe) {
	//Use this yaw only rotation to set the direction of the base (and thus end effector)
	Eigen::Matrix3d br_sp = extract_yaw_component(ge_sp.linear());

	//Construct the true end effector
	Eigen::Affine3d sp = Eigen::Affine3d::Identity();
	sp.linear() = br_sp*gbe.linear();
	sp.translation() = ge_sp.translation();

	//Use the inverse transform to get the true base transform
	return sp*gbe.inverse();
}

Eigen::Vector3d MantisPath::calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::VectorXd &vbe) {
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

Eigen::Matrix3d MantisPath::extract_yaw_component(const Eigen::Matrix3d r) {
	Eigen::Vector3d sp_x = Eigen::Vector3d::UnitX();
	Eigen::Vector3d sp_y = Eigen::Vector3d::UnitY();

	//As long as y isn't straight up
	if(r.col(1) != Eigen::Vector3d::UnitZ()) {
		//If we have ||roll|| > 90Deg
		Eigen::Vector3d y_c;
		if(r(2,2) > 0.0) {
			y_c = r.col(1);
		} else {
			y_c = -r.col(1);
		}

		sp_x = y_c.cross(Eigen::Vector3d::UnitZ());
		sp_y = Eigen::Vector3d::UnitZ().cross(sp_x);
	} else { //Use X-axis for the edge-case
		Eigen::Vector3d x_c = r.col(0);

		sp_y = Eigen::Vector3d::UnitZ().cross(x_c);
		sp_x = sp_y.cross(Eigen::Vector3d::UnitZ());
	}

	//Get the pure yaw rotation
	Eigen::Matrix3d ry;
	ry << sp_x.normalized(),
		  sp_y.normalized(),
		  Eigen::Vector3d::UnitZ();

	return ry;
}
