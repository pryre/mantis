#include <ros/ros.h>

#include <forward_kinematics/forward_kinematics.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

//#include <eigen3/Eigen/Dense>
#include <math.h>

//TODO: Add goal joints

ForwardKinematics::ForwardKinematics() :
	nh_("~"),
	param_model_name_("mantis_uav"),
	param_arm_len_(0.25),
	param_do_viz_(true),
	param_done_viz_(false) {

	pub_viz_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization", 10, true);

	sub_state_odom_ = nh_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &ForwardKinematics::callback_state_odom, this );
	sub_state_joints_ = nh_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &ForwardKinematics::callback_state_joints, this );

	//TODO: params
	param_mount_translation_ = tf2::Vector3(0.0,0.0,-0.05);
	tf2::Quaternion mount_align;
	tf2::Quaternion mount_rot;
	mount_align.setRPY(M_PI/2.0,0.0,0.0);
	mount_rot.setRPY(0.0,0.0,-M_PI/2.0);
	param_mount_rotation_ = mount_align*mount_rot;
	//param_arm_len_ = ...

	ROS_INFO("Configuring static mounts");
	base_arm_rot(param_mount_translation_, param_mount_rotation_);

	ROS_INFO("Forward kinematics ready");
}

ForwardKinematics::~ForwardKinematics() {
}

void ForwardKinematics::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	geometry_msgs::TransformStamped t;

	t.header.frame_id = msg_in->header.frame_id;
	t.header.stamp = msg_in->header.stamp;
	t.child_frame_id = param_model_name_;

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
	param_joint_names_ = msg_in->name;

	for(int i=0; i<param_joint_names_.size(); i++) {
		geometry_msgs::TransformStamped t;

		if(i==0) {
			t.header.frame_id = "arm_mount";
			t.transform.translation.x = 0.0;
		} else {
			t.header.frame_id = msg_in->name[i-1];
			t.transform.translation.x = param_arm_len_;
		}

		t.header.stamp = msg_in->header.stamp;
		t.child_frame_id = msg_in->name[i];

		t.transform.translation.z = 0.0;
		t.transform.translation.y = 0.0;
		tf2::Quaternion q;
		q.setRPY(0.0,0.0,msg_in->position[i]);
		t.transform.rotation.w = q.w();
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();

		tfbr_.sendTransform(t);
	}

	if(param_do_viz_ && !param_done_viz_)
		do_viz(&(msg_in->name));
}

void ForwardKinematics::base_arm_rot( tf2::Vector3 p, tf2::Quaternion q ) {
	geometry_msgs::TransformStamped t;

	t.header.frame_id = param_model_name_;
	t.header.stamp = ros::Time::now();
	t.child_frame_id = "arm_mount";

	t.transform.translation.x = p.x();
	t.transform.translation.y = p.y();
	t.transform.translation.z = p.z();
	t.transform.rotation.w = q.w();
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();

	tfsbr_.sendTransform(t);
}

void ForwardKinematics::do_viz( const std::vector<std::string> *arm_names ) {
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

	param_done_viz_ = true;
}
