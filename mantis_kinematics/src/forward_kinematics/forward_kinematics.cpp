#include <ros/ros.h>

#include <forward_kinematics/forward_kinematics.h>
#include <forward_kinematics/dh_gen.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_broadcaster.h>
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
	nh_("~"),
	param_model_id_("mantis_uav"),
	param_do_viz_(true),
	param_done_viz_(false) {

	//Load parameters
	nh_.param("model_id", param_model_id_, param_model_id_);

	nh_.getParam("links/var", param_dh_joints_);

	ROS_INFO("Loading %li links...", param_dh_joints_.size());
	bool success = true;

	for(int i=0; i<param_dh_joints_.size(); i++) {
		//Clean up dh if needed
		if( (param_dh_joints_[i] > 4) ||  (param_dh_joints_[i] < 0) ) {
			ROS_FATAL("Joint selector for link %i is not valid (0 <= %i <= 4)", i, param_dh_joints_[i]);
			success = false;
			break;
		}

		std::vector<double> dh;

		if(nh_.getParam("links/l" + std::to_string(i), dh)) {
			param_dh_params_.push_back(dh);
		} else {
			ROS_FATAL("Could not find DH parameters for link %i", i);
			success = false;
			break;
		}
	}

	if(success) {
		ROS_INFO("All links loaded!");

		//Configure publishers and subscribers
		pub_viz_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization", 10, true);

		sub_state_odom_ = nh_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &ForwardKinematics::callback_state_odom, this );
		sub_state_joints_ = nh_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &ForwardKinematics::callback_state_joints, this );

		ROS_INFO("Configuring static mounts...");
		configure_static_joints();

		ROS_INFO("Forward kinematics running!");
	} else {
		ros::shutdown();
	}
}

ForwardKinematics::~ForwardKinematics() {
}

void ForwardKinematics::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
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
}

void ForwardKinematics::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	int joint_counter = 0;

	for(int i=0; i<param_dh_joints_.size(); i++) {
		int jt = param_dh_joints_[i];

		Eigen::Affine3d g;
		double d = param_dh_params_[i][0];
		double t = param_dh_params_[i][1];
		double r = param_dh_params_[i][2];
		double a = param_dh_params_[i][3];
		double j = 0.0;

		if(jt > 0) {
			if(joint_counter >= msg_in->position.size()) {
				ROS_ERROR("Tried to read index (%i) but joint message is %li long", joint_counter, msg_in->position.size());
			} else {
				j = msg_in->position[joint_counter];
				joint_counter++;
			}
		}

		switch(jt) {
			case 1: {
				dh_gen(g, d + j, t, r, a);
				break;
			}
			case 2: {
				dh_gen(g, d, t + j, r, a);
				break;
			}
			case 3: {
				dh_gen(g, d, t, r + j, a);
				break;
			}
			case 4: {
				dh_gen(g, d, t, r, a + j);
				break;
			}
			default: {
				//Handled by static transforms
				//dh_gen(g, d, t, r, a);
			}
		}

		geometry_msgs::TransformStamped transform = tf2::eigenToTransform(g);
		transform.header.stamp = msg_in->header.stamp;
		transform.header.frame_id = (i == 0) ? param_model_id_ : param_model_id_ + "/l" + std::to_string(i-1);
		transform.child_frame_id = param_model_id_ + "/l" + std::to_string(i);

		tfbr_.sendTransform(transform);
	}

	//if(param_do_viz_ && !param_done_viz_)
	//	do_viz(&(msg_in->name));
}

void ForwardKinematics::configure_static_joints() {
	ros::Time stamp = ros::Time::now();

	for(int i=0; i<param_dh_joints_.size(); i++) {
		if(param_dh_joints_[i] == 0) {
			Eigen::Affine3d g;
			double d = param_dh_params_[i][0];
			double t = param_dh_params_[i][1];
			double r = param_dh_params_[i][2];
			double a = param_dh_params_[i][3];

			dh_gen(g, d, t, r, a);

			ROS_INFO("[d,t,r,a]: [%0.4f, %0.4f, %0.4f, %0.4f]", d,t,r,a);
			ROS_INFO_STREAM("g:" << std::endl << g.matrix());

			geometry_msgs::TransformStamped tf = tf2::eigenToTransform(g);
			tf.header.stamp = stamp;
			tf.header.frame_id = (i == 0) ? param_model_id_ : param_model_id_ + "/l" + std::to_string(i-1);
			tf.child_frame_id = param_model_id_ + "/l" + std::to_string(i);
			//tf.transform = affine3dToTransform(g);

			tfsbr_.sendTransform(tf);
		}
	}
}

geometry_msgs::Transform ForwardKinematics::affine3dToTransform( Eigen::Affine3d &g ) {
	geometry_msgs::Transform tf;

	Eigen::Quaterniond q(g.linear());
	q.normalize();

	tf.translation.x = g.translation()(0);
	tf.translation.y = g.translation()(1);
	tf.translation.z = g.translation()(2);
	tf.rotation.w = q.w();
	tf.rotation.x = q.x();
	tf.rotation.y = q.y();
	tf.rotation.z = q.z();

	return tf;
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
