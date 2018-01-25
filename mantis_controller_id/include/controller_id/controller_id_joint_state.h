#pragma once

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <vector>

class ControllerIDJointState {
	private:
		int num_joints_;

		Eigen::VectorXd q_;
		Eigen::VectorXd qd_;
		Eigen::VectorXd qdd_;

		double beta_;

	private:
		double lpf(const double v, const double vp);

	public:
		ControllerIDJointState( ros::NodeHandle *nh, const int num_joints, const double accel_filter );

		~ControllerIDJointState( void );

		void update_states(std::vector<double> q, std::vector<double> qd);

		Eigen::VectorXd q( void );
		Eigen::VectorXd qd( void );
		Eigen::VectorXd qdd( void );
};
