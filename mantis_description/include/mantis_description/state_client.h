#pragma once

#include <ros/ros.h>

#include <mantis_msgs/State.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <eigen3/Eigen/Dense>

class MantisStateClient {
	private:
		ros::NodeHandle *nh_;

		ros::Subscriber sub_state_;

		ros::Time timestamp_;

		Eigen::Affine3d g_;
		Eigen::Vector3d bv_;
		Eigen::Vector3d ba_;
		Eigen::Vector3d bw_;
		Eigen::Vector3d bwa_;

		Eigen::VectorXd r_;
		Eigen::VectorXd rd_;
		Eigen::VectorXd rdd_;

		double voltage_;
		bool flight_ready_;

	public:
		MantisStateClient( ros::NodeHandle *nh );

		~MantisStateClient( void );

		ros::Time time_updated( void );

		Eigen::Affine3d g( void );
		Eigen::Vector3d bv( void );
		Eigen::Vector3d wv( void );
		Eigen::Vector3d bw( void );
		Eigen::Vector3d ba( void );
		Eigen::Vector3d bwa( void );
		Eigen::VectorXd r( void );
		Eigen::VectorXd rd( void );
		Eigen::VectorXd rdd( void );
		double voltage( void );
		bool flight_ready( void );

		bool ok( void );

	private:
		void callback_state(const mantis_msgs::State::ConstPtr &msg_in);

		//Conversion Helpers
		Eigen::Vector3d vector_from_msg(const geometry_msgs::Vector3 v);
		Eigen::Vector3d point_from_msg(const geometry_msgs::Point p);
		Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);
};
