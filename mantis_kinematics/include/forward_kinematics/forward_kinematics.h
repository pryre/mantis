#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>

class ForwardKinematics {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_viz_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_joints_;

		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfsbr_;

		std::string param_model_id_;
		std::vector<int> param_dh_joints_;
		std::vector<std::vector<double>> param_dh_params_;

		bool param_do_viz_;
		bool param_done_viz_;

	public:
		ForwardKinematics( void );

		~ForwardKinematics( void );

		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
		void configure_static_joints( void );
		geometry_msgs::Transform affine3dToTransform( Eigen::Affine3d &g );
		void do_viz( const std::vector<std::string> *arm_names );
};
