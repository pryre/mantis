#pragma once

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

class ForwardKinematics {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_viz_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_joints_;

		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfsbr_;

		std::string param_model_name_;
		std::vector<std::string> param_joint_names_;
		double param_arm_len_;
		bool param_do_viz_;
		bool param_done_viz_;
		tf2::Vector3 param_mount_translation_;
		tf2::Quaternion param_mount_rotation_;

	public:
		ForwardKinematics( void );

		~ForwardKinematics( void );

		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);

		void base_arm_rot( tf2::Vector3 p, tf2::Quaternion q );
		void do_viz( const std::vector<std::string> *arm_names );
};
