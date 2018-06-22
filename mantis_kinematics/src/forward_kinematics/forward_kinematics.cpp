#include <ros/ros.h>

#include <forward_kinematics/forward_kinematics.h>

#include <dh_parameters/dh_parameters.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>

#include <stdio.h>

//TODO: Add goal joints

ForwardKinematics::ForwardKinematics() :
	nh_(),
	nhp_("~"),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_do_end_effector_pose_(true),
	param_do_viz_(true),
	param_done_viz_(false),
	tfBuffer_(ros::Duration(20.0)) {

	int num_links = 0;

	//Load parameters
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("do_viz", param_do_viz_, param_do_viz_);
	nhp_.param("end_effector_pose", param_do_end_effector_pose_, param_do_end_effector_pose_);

	nh_.param("body/num", num_links, num_links);
	ROS_INFO("Loading %i links...", num_links);

	bool success = true;

	for(int i=0; i<num_links; i++) {
		DHParameters dh( &nh_, "body/b" + std::to_string(i) + "/link");

		if( dh.is_valid() ) {
			param_joints_.push_back(dh);
		} else {
			ROS_FATAL("Error loading joint %i", i);
			success = false;
			break;
		}
	}

	if(success) {
		ROS_INFO("All links loaded!");

		//Configure publishers and subscribers
		pub_end_ = nh_.advertise<geometry_msgs::PoseStamped>("pose/end_effector", 10);
		pub_viz_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization", 10, true);

		sub_state_odom_ = nh_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &ForwardKinematics::callback_state_odom, this );
		sub_state_joints_ = nh_.subscribe<sensor_msgs::JointState>( "joint_states", 10, &ForwardKinematics::callback_state_joints, this );

		ROS_INFO("Configuring static mounts...");
		do_reset();

		ROS_INFO("Forward kinematics running!");
	} else {
		ros::shutdown();
	}
}

ForwardKinematics::~ForwardKinematics() {
}

void ForwardKinematics::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	check_update_time();

	geometry_msgs::TransformStamped t;

	t.header.frame_id = msg_in->header.frame_id;
	t.header.stamp = msg_in->header.stamp;
	t.child_frame_id = param_model_id_;

	t.transform.translation.x = msg_in->pose.pose.position.x;
	t.transform.translation.y = msg_in->pose.pose.position.y;
	t.transform.translation.z = msg_in->pose.pose.position.z;
	t.transform.rotation.w = msg_in->pose.pose.orientation.w;
	t.transform.rotation.x = msg_in->pose.pose.orientation.x;
	t.transform.rotation.y = msg_in->pose.pose.orientation.y;
	t.transform.rotation.z = msg_in->pose.pose.orientation.z;

	tfbr_.sendTransform(t);
	tfBuffer_.setTransform(t, ros::this_node::getName());	//Set the base link
}

void ForwardKinematics::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	check_update_time();

	for(int i=0; i<param_joints_.size(); i++) {
		if( param_joints_[i].jt() != DHParameters::JointType::Static ) {
			for(int j=0; j<msg_in->name.size(); j++) {
				if(param_joints_[i].name() == msg_in->name[j]) {
					param_joints_[i].update(msg_in->position[j]);

					geometry_msgs::TransformStamped transform = tf2::eigenToTransform(param_joints_[i].transform());
					transform.header.stamp = msg_in->header.stamp;
					transform.header.frame_id = (i == 0) ? param_model_id_ : param_model_id_ + "/link_" + std::to_string(i);
					transform.child_frame_id = param_model_id_ + "/link_" + std::to_string(i+1);

					tfbr_.sendTransform(transform);
					tfBuffer_.setTransform(transform, ros::this_node::getName());	//Set the internal dynamic joints
				}
			}
		}
	}

	//Send out the end effector pose
	if(param_do_end_effector_pose_) {
		geometry_msgs::PoseStamped msg_end_out_;
	    geometry_msgs::TransformStamped tf_end;
		std::string last_joint = param_model_id_ + "/link_" + std::to_string(param_joints_.size());

		try {
			tf_end = tfBuffer_.lookupTransform(param_frame_id_, last_joint, ros::Time(0));

			msg_end_out_.header = tf_end.header;
			msg_end_out_.pose.position.x = tf_end.transform.translation.x;
			msg_end_out_.pose.position.y = tf_end.transform.translation.y;
			msg_end_out_.pose.position.z = tf_end.transform.translation.z;
			msg_end_out_.pose.orientation.w = tf_end.transform.rotation.w;
			msg_end_out_.pose.orientation.x = tf_end.transform.rotation.x;
			msg_end_out_.pose.orientation.y = tf_end.transform.rotation.y;
			msg_end_out_.pose.orientation.z = tf_end.transform.rotation.z;

			pub_end_.publish(msg_end_out_);
		}
		catch(const tf2::LookupException& e) {
			ROS_WARN_THROTTLE(1.0, "Lookup error: %s", e.what());
		}
		catch(const tf2::ExtrapolationException& e) {
			ROS_WARN_THROTTLE(1.0, "Extrapolation error: %s", e.what());
		}
	}
	//if(param_do_viz_ && !param_done_viz_)
	//	do_viz(&(msg_in->name));
}

void ForwardKinematics::configure_static_joints() {
	ros::Time stamp = ros::Time::now();

	for(int i=0; i<param_joints_.size(); i++) {
		if( param_joints_[i].jt() == DHParameters::JointType::Static ) {
			geometry_msgs::TransformStamped tf = tf2::eigenToTransform( param_joints_[i].transform() );
			tf.header.stamp = stamp;
			tf.header.frame_id = (i == 0) ? param_model_id_ : param_model_id_ + "/link_" + std::to_string(i);
			tf.child_frame_id = param_model_id_ + "/link_" + std::to_string(i+1);

			tfsbr_.sendTransform(tf);
			tfBuffer_.setTransform(tf, ros::this_node::getName(), true);	//Set the internal static joints
		}
	}
}

void ForwardKinematics::do_reset() {
	tfBuffer_.clear();

	configure_static_joints();

	time_last_update_ = ros::Time::now();
}

void ForwardKinematics::check_update_time() {
	if(ros::Time::now() < time_last_update_) {
		ROS_INFO("Detected jump back in time, reseting transforms");
		do_reset();
	} else {
		time_last_update_ = ros::Time::now();
	}
}


void ForwardKinematics::do_viz( const std::vector<std::string> *arm_names ) {
	/*
	visualization_msgs::MarkerArray markers;
	ros::Time stamp = ros::Time::now();

	visualization_msgs::Marker m;
	m.header.stamp = stamp;
	m.ns = param_model_name_;
	m.action = m.ADD;
	m.color.a = 1.0;
	m.lifetime = ros::Duration(0);
	m.frame_locked = true;

	//Body
	//TODO: params
	double frame_h = 0.05;
	double frame_w = 0.55;

	m.header.frame_id = param_model_name_;
	m.id = 0;
	m.type = m.CYLINDER;
	m.pose.position.x = 0.0;
	m.pose.position.y = 0.0;
	m.pose.position.z = frame_h/2;
	m.pose.orientation.w = 1.0;
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.scale.x = frame_w;
	m.scale.y = frame_w;
	m.scale.z = frame_h;
	m.color.r = 0.8;
	m.color.g = 0.8;
	m.color.b = 0.8;

	markers.markers.push_back(m);

	//Arms
	for(int i=0; i<arm_names->size(); i++) {
		double arm_w = 0.05;

		m.header.frame_id = (*arm_names)[i];
		m.id++;
		m.type = m.CYLINDER;
		m.pose.position.x = param_arm_len_/2;
		m.pose.position.y = 0.0;
		m.pose.position.z = 0.0;
		m.pose.orientation.w = 0.7071;
		m.pose.orientation.x = 0.0;
		m.pose.orientation.y = 0.7071;
		m.pose.orientation.z = 0.0;
		m.scale.x = arm_w;
		m.scale.y = arm_w;
		m.scale.z = param_arm_len_;
		m.color.r = 0.8;
		m.color.g = 0.8;
		m.color.b = 0.8;

		markers.markers.push_back(m);
	}

	pub_viz_.publish(markers);
	*/
	param_done_viz_ = true;
}
