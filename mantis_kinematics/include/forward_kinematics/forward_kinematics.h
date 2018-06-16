#pragma once

#include <ros/ros.h>

#include <dh_parameters/dh_parameters.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>

class ForwardKinematics {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Publisher pub_end_;
		ros::Publisher pub_viz_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_joints_;

		tf2_ros::Buffer tfBuffer_;
		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfsbr_;

		std::string param_frame_id_;
		std::string param_model_id_;
		std::vector<DHParameters> param_joints_;

		bool param_do_end_effector_pose_;
		bool param_do_viz_;
		bool param_done_viz_;

		ros::Time time_last_update_;

	public:
		ForwardKinematics( void );

		~ForwardKinematics( void );

		void do_reset();
		void check_update_time();

		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
		void configure_static_joints( void );
		void do_viz( const std::vector<std::string> *arm_names );
};
