/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>

#include <mantis_params/param_client.h>

#include <eigen3/Eigen/Dense>

namespace MantisState {

//Q=(q^2)[T^3/3, T^2/2; T^2/2, T]
//	T: sampling time
//	q: gain scailing for sensor
typedef struct {
	ros::Time stamp;

	bool data_valid;
	uint64_t count;

	double exp_rate;

	double avg_rate;
	double avg_sampling_time;
} sensor_info_t;

typedef struct {
	sensor_info_t info;
	geometry_msgs::Vector3 linear_acceleration;
} sensor_data_accel_t;

typedef struct {
	sensor_info_t info;
	geometry_msgs::Vector3 angular_velocity;
} sensor_data_gyro_t;

typedef struct {
	sensor_info_t info;
	geometry_msgs::Vector3 magnetic_field;
} sensor_data_mag_t;

typedef struct {
	sensor_info_t info;
	geometry_msgs::Quaternion attitude;
} sensor_data_att_t;

typedef struct {
	sensor_info_t info;
	geometry_msgs::Point position;
} sensor_data_pos_t;

typedef struct {
	sensor_info_t info;
	geometry_msgs::Vector3 linear_vel;
} sensor_data_bvel_t;

typedef struct {
	sensor_info_t info;
	uint64_t num;
	Eigen::VectorXd positions;
	Eigen::VectorXd velocities;
} sensor_data_joints_t;

typedef struct {
	sensor_info_t info;
	double voltage;
} sensor_data_battery_t;

typedef struct {
	sensor_data_accel_t accel;
	sensor_data_gyro_t gyro;
	sensor_data_mag_t mag;
	sensor_data_att_t att;
	sensor_data_pos_t pos;
	sensor_data_bvel_t bvel;
	sensor_data_joints_t joints;
	sensor_data_battery_t battery;
} sensor_readings_t;

typedef struct {
	double rate_deviation;

	double rate_accel;
	double rate_gyro;
	double rate_mag;
	double rate_att;
	double rate_pos;
	double rate_vel;
	double rate_joints;
	double rate_battery;

	double att_adpt_bias;
	double att_alpha_accel;
	double att_alpha_gyro;
	double att_corr_accel;
	double att_corr_mag;
	double att_corr_ext;

	double battery_alpha_voltage;

	double scaling_accel;
	double scaling_pos;
	double scaling_vel;
	double scaling_joints;
} sensor_params_t;

typedef double T;

class Server {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_estimator_;

		ros::Subscriber sub_state_imu_;
		ros::Subscriber sub_state_mav_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_battery_;
		ros::Subscriber sub_state_joints_;

		ros::Publisher pub_state_;

		MantisParams::Client p_;

		//Node Parameters
		double param_rate_;
		ros::Time timestamp_config_change_;

		//Sensor Data
		sensor_readings_t sensor_data_;
		sensor_params_t sensor_params_;

		uint16_t status_updated_fields_;
		uint16_t status_sensor_health_;

		//Current Estimated state
		/*
		typedef MantisRobot::State<T> State_;
		typedef MantisRobot::Control<T> Control_;
		typedef MantisRobot::SystemModel<T> SystemModel_;

		typedef MantisRobot::PositionMeasurement<T> PositionMeasurement_;
		typedef MantisRobot::OrientationMeasurement<T> OrientationMeasurement_;
		typedef MantisRobot::PositionMeasurementModel<T> PositionModel_;
		typedef MantisRobot::OrientationMeasurementModel<T> OrientationModel_;
		*/

		Eigen::Affine3d g_;
		Eigen::Vector3d bv_;
		Eigen::Vector3d ba_;
		Eigen::Vector3d bw_;

		Eigen::VectorXd r_;
		Eigen::VectorXd rd_;

		double voltage_;

		//Arming check
		bool mav_ready_;

	public:
		Server( void );

		~Server( void );

	private:
		void callback_estimator(const ros::TimerEvent& e);

		void update_timing_data(sensor_info_t& info, const ros::Time& stamp);

		/*
		void update_g( const Eigen::Affine3d &g );
		void update_bw( const Eigen::Vector3d &bw );
		void update_bw( const Eigen::Vector3d &bw, const double dt );
		void update_bv( const Eigen::Vector3d &bv );
		void update_ba( const Eigen::Vector3d &ba );
		void update_r( const Eigen::VectorXd &r );
		void update_rd( const Eigen::VectorXd &rd );
		void update_rdd( const Eigen::VectorXd &rdd );
		void update_voltage( const double voltage );
		void update_mav_ready( const bool ready );
		*/

		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in);
		void callback_state_mav(const mavros_msgs::State::ConstPtr& msg_in);
		void callback_state_battery(const sensor_msgs::BatteryState::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
};
};
