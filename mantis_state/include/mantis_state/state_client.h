/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

#include <ros/ros.h>

#include <mantis_msgs/State.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <mantis_params/param_client.h>

#include <eigen3/Eigen/Dense>
#include <string>

namespace MantisState {
class Client {
	private:
		ros::NodeHandle nh_;
		MantisParams::Client& p_;

		ros::Subscriber sub_state_;

		std::string frame_id_;
		std::string child_frame_id_;
		ros::Time timestamp_;
		ros::Time configuration_stamp_;

		uint16_t status_updated_fields_;
		uint16_t status_sensor_health_;

		Eigen::Affine3d g_;
		Eigen::Vector3d bv_;
		Eigen::Vector3d ba_;
		Eigen::Vector3d bw_;
		//Eigen::Vector3d bwa_;

		Eigen::VectorXd r_;
		Eigen::VectorXd rd_;
		//Eigen::VectorXd rdd_;

		double voltage_;
		bool mav_safety_disengaged_;
		bool mav_armed_;

	public:
		Client( const ros::NodeHandle& nh, MantisParams::Client& p );

		~Client( void );

		const ros::Time& time_updated( void );
		const ros::Time& state_configuration_stamp( void );

		bool wait_for_state( void );

		bool ok( void );
		bool flight_ready( void );

		const std::string& frame_id( void );
		const std::string& model_id( void );

		const Eigen::Affine3d& g( void );
		const Eigen::Vector3d& bv( void );
		Eigen::Vector3d wv( void );
		const Eigen::Vector3d& bw( void );
		const Eigen::Vector3d& ba( void );
		//const Eigen::Vector3d& bwa( void );
		const Eigen::VectorXd& r( void );
		const Eigen::VectorXd& rd( void );
		//const Eigen::VectorXd& rdd( void );
		const double& voltage( void );
		const bool& mav_safety_disengaged( void );
		const bool& mav_safety_armed( void );

	private:
		void callback_state(const mantis_msgs::State::ConstPtr &msg_in);
};
};
