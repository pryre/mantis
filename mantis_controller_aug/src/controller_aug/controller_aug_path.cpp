#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <controller_aug/controller_aug_path.h>
#include <controller_aug/controller_aug_params.h>
#include <controller_aug/se_tools.h>
#include <controller_aug/dynamics/calc_Dq.h>
#include <controller_aug/dynamics/calc_Cqqd.h>
#include <controller_aug/dynamics/calc_Lqd.h>
#include <controller_aug/dynamics/calc_Je.h>
#include <dh_parameters/dh_parameters.h>
#include <contrail/path_extract.h>
#include <mantis_controller_aug/ControlParamsConfig.h>

#include <sensor_msgs/JointState.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

ControllerAugPath::ControllerAugPath() :
	nh_(),
	nhp_("~"),
	p_(&nh_, &nhp_),
	ref_path_(&nhp_),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_path_rate_(50.0),
	dyncfg_control_settings_(ros::NodeHandle(nhp_, "control_settings")) {

	bool success = true;

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("path_rate", param_path_rate_, param_path_rate_);

	dyncfg_control_settings_.setCallback(boost::bind(&ControllerAugPath::callback_cfg_control_settings, this, _1, _2));

	//Load the robot parameters
	p_.load();	//TODO have this give a success if loaded correctly

	//Load in the link definitions
	for(int i=0; i<p_.body_num; i++) {
		DHParameters dh( &nh_, "body/b" + std::to_string(i) + "/link" );

		if( dh.is_valid() ) {
			joints_.push_back(dh);
		} else {
			ROS_FATAL("Error loading joint %i", int(i));
			success = false;
			break;
		}
	}

	state_.init(p_.manip_num);	//Initialize the state container

	if(success) {
		ROS_INFO( "Loaded configuration for %i links", int(joints_.size()) );

		pub_traj_ = nhp_.advertise<nav_msgs::Odometry>("output/traj", 10);
		sub_state_odom_ = nhp_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &ControllerAugPath::callback_state_odom, this );
		sub_state_joints_ = nhp_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &ControllerAugPath::callback_state_joints, this );

		timer_path_ = nhp_.createTimer(ros::Duration(1.0/param_path_rate_), &ControllerAugPath::callback_path, this );

		ROS_INFO_ONCE("Augmented path control loaded!");
	} else {
		ROS_WARN("Augmented path control shutting down.");
		ros::shutdown();
	}
}

ControllerAugPath::~ControllerAugPath() {
}

void ControllerAugPath::callback_cfg_control_settings(mantis_controller_aug::ControlParamsPathConfig &config, uint32_t level) {
	param_track_end_ = config.track_end;
	param_accurate_end_tracking_ = config.accurate_end_tracking;
}

void ControllerAugPath::callback_path(const ros::TimerEvent& e) {
	double dt = (e.current_real - e.last_real).toSec();

	//Goal States

	if( ( ref_path_.has_valid_path() || ref_path_.has_valid_fallback() ) &&
		( msg_odom_tr_ != ros::Time(0) ) &&
		( msg_joints_tr_ != ros::Time(0) ) ) {

		Eigen::Affine3d ge_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gev_sp = Eigen::Vector3d::Zero();

		ref_path_.get_ref_state(ge_sp, gev_sp, e.current_real);

		if( param_track_end_ ) {
			//Compute transform for all joints in the chain to get base to end effector
			Eigen::Affine3d gbe = Eigen::Affine3d::Identity();
			Eigen::MatrixXd Je = Eigen::MatrixXd::Zero(6, state_.rd().size());

			//This hole thing could be done dynamically for any joint
			//Compute transform for all joints in the chain to get base to end effector
			for(int i=0; i<joints_.size(); i++) {
				gbe = gbe * joints_[i].transform();
			}

			//The linear velocity of the base is dependent on the joint velocities the end effector
			//End effector velocity jacobian
			//XXX: Pretty sure this can be done dynamically in dh_parameters
			calc_Je(Je,
					joints_[1].r(),
					joints_[2].r(),
					joints_[2].q());

			//Calculate the tracking offset for the base
			state_.update_g_sp( calc_goal_base_transform(ge_sp, gbe) );

			//Compensate for velocity terms in manipulator movement
			//This is more accurate, but can make things more unstable
			if(param_accurate_end_tracking_) {
				state_.update_gv_sp( calc_goal_base_velocity(gev_sp, state_.g().linear()*gbe.linear(), Je, state_.rd()) );
			} else {
				state_.update_gv_sp( gev_sp );
			}
		} else {
			//Just track the robot base
			state_.update_g_sp( ge_sp );
			state_.update_gv_sp( gev_sp );
		}

		Eigen::Vector3d gv_sp_b = state_.g_sp().linear()*state_.gv_sp();

		nav_msgs::Odometry msg_traj_out;
		msg_traj_out.header.stamp = e.current_real;
		msg_traj_out.header.frame_id = param_frame_id_;
		msg_traj_out.pose.pose = pose_from_eig(state_.g_sp());
		msg_traj_out.twist.twist.linear = vector_from_eig(gv_sp_b);

		pub_traj_.publish(msg_traj_out);
	}
}

Eigen::Vector3d ControllerAugPath::position_from_msg(const geometry_msgs::Point p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond ControllerAugPath::quaternion_from_msg(const geometry_msgs::Quaternion q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Affine3d ControllerAugPath::affine_from_msg(const geometry_msgs::Pose pose) {
		Eigen::Affine3d a;

		a.translation() << position_from_msg(pose.position);
		a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

		return a;
}

geometry_msgs::Vector3 ControllerAugPath::vector_from_eig(const Eigen::Vector3d &v) {
	geometry_msgs::Vector3 vec;

	vec.x = v.x();
	vec.y = v.y();
	vec.z = v.z();

	return vec;
}

geometry_msgs::Point ControllerAugPath::point_from_eig(const Eigen::Vector3d &p) {
	geometry_msgs::Point point;

	point.x = p.x();
	point.y = p.y();
	point.z = p.z();

	return point;
}

geometry_msgs::Quaternion ControllerAugPath::quaternion_from_eig(const Eigen::Quaterniond &q) {
	geometry_msgs::Quaternion quat;
	Eigen::Quaterniond qn = q.normalized();

	quat.w = qn.w();
	quat.x = qn.x();
	quat.y = qn.y();
	quat.z = qn.z();

	return quat;
}

geometry_msgs::Pose ControllerAugPath::pose_from_eig(const Eigen::Affine3d &g) {
	geometry_msgs::Pose pose;

	pose.position = point_from_eig(g.translation());
	pose.orientation = quaternion_from_eig(Eigen::Quaterniond(g.linear()));

	return pose;
}

Eigen::Affine3d ControllerAugPath::calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe) {
	//Use this yaw only rotation to set the direction of the base (and thus end effector)
	Eigen::Matrix3d br_sp = extract_yaw_component(ge_sp.linear());

	//Construct the true end effector
	Eigen::Affine3d sp = Eigen::Affine3d::Identity();
	sp.linear() = br_sp*gbe.linear();
	sp.translation() = ge_sp.translation();

	//Use the inverse transform to get the true base transform
	return sp*gbe.inverse();
}

Eigen::Vector3d ControllerAugPath::calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::MatrixXd &Je, const Eigen::VectorXd &rd) {
	//Velocity of the end effector in the end effector frame
	Eigen::VectorXd vbe = Je*rd;

	//Eigen::Affine3d Vbe;
	//Vbe.translation() << vbe.segment(0,3);
	//Vbe.linear() << vee_up(vbe.segment(3,3));

	//Get linear velocity in the world frame
	Eigen::Vector3d Ve = Re*vbe.segment(0,3);

	//Only care about the translation movements
	return gev_sp - Ve;
}

Eigen::Matrix3d ControllerAugPath::extract_yaw_component(const Eigen::Matrix3d r) {
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

void ControllerAugPath::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_odom_tr_).toSec();
	msg_odom_tr_ = msg_in->header.stamp;

	//Update pose
	Eigen::Affine3d g = affine_from_msg(msg_in->pose.pose);
	state_.update_g( g );

	//Update body velocities
	Eigen::Vector3d bv(msg_in->twist.twist.linear.x,
					   msg_in->twist.twist.linear.y,
					   msg_in->twist.twist.linear.z);
	state_.update_bv( bv );

	Eigen::Vector3d bw = Eigen::Vector3d(msg_in->twist.twist.angular.x,
										 msg_in->twist.twist.angular.y,
										 msg_in->twist.twist.angular.z);

	state_.update_bw( bw, dt );
}

void ControllerAugPath::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_joints_tr_).toSec();

	//Only updating if dt is acceptable
	if( (dt > 0.0) && (dt < 1.0) ) {
		for(int i=0; i<joints_.size(); i++) {
			//Only update the dynamic links
			if(joints_[i].jt() != DHParameters::JointType::Static) {
				for(int j=0; j<msg_in->name.size(); j++) {
					//If we have the right joint
					if(joints_[i].name() == msg_in->name[j])
						joints_[i].update(msg_in->position[j], msg_in->velocity[j], dt);
				}
			}
		}
	}

	int jc = 0;
	Eigen::VectorXd r(state_.r().size());
	Eigen::VectorXd rd(state_.rd().size());
	Eigen::VectorXd rdd(state_.rdd().size());

	for(int i=0; i<joints_.size(); i++) {
		//Only update the dynamic links
		if(joints_[i].jt() != DHParameters::JointType::Static) {
			r(jc) = joints_[i].q();
			rd(jc) = joints_[i].qd();
			rdd(jc) = joints_[i].qdd();

			jc++;
		}
	}

	state_.update_r(r);
	state_.update_rd(rd);
	state_.update_rdd(rdd);

	msg_joints_tr_ = msg_in->header.stamp;
}
