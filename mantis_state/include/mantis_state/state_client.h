#pragma once

#include <ros/ros.h>

#include <mantis_msgs/State.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <mantis_params/param_client.h>

#include <eigen3/Eigen/Dense>

class MantisStateClient {
	private:
		ros::NodeHandle nh_;
		MantisParamClient& p_;

		ros::Subscriber sub_state_;

		ros::Time timestamp_;
		ros::Time configuration_stamp_;

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
		MantisStateClient( const ros::NodeHandle& nh, MantisParamClient& p );

		~MantisStateClient( void );

		const ros::Time& time_updated( void );
		const ros::Time& state_configuration_stamp( void );

		bool wait_for_state( void );

		const Eigen::Affine3d& g( void );
		const Eigen::Vector3d& bv( void );
		Eigen::Vector3d wv( void );
		const Eigen::Vector3d& bw( void );
		const Eigen::Vector3d& ba( void );
		const Eigen::Vector3d& bwa( void );
		const Eigen::VectorXd& r( void );
		const Eigen::VectorXd& rd( void );
		const Eigen::VectorXd& rdd( void );
		const double& voltage( void );
		const bool& flight_ready( void );

		bool ok( void );

	private:
		void callback_state(const mantis_msgs::State::ConstPtr &msg_in);
};
