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

		bool param_use_odom_avel_;
		double param_rate_;

		ros::Time msg_odom_tr_;
		ros::Time msg_battery_tr_;
		ros::Time msg_joints_tr_;
		ros::Time msg_imu_tr_;
		ros::Time msg_mav_state_tr_;

		ros::Time time_last_est_;

		uint16_t status_updated_fields_;
		uint16_t status_sensor_health_;

		Eigen::Affine3d g_;
		Eigen::Vector3d bv_;
		Eigen::Vector3d ba_;
		Eigen::Vector3d bw_;
		Eigen::Vector3d bwa_;

		Eigen::VectorXd r_;
		Eigen::VectorXd rd_;
		Eigen::VectorXd rdd_;

		double voltage_;
		bool mav_ready_;

	public:
		Server( void );

		~Server( void );

	private:
		void callback_estimator(const ros::TimerEvent& e);

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

		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in);
		void callback_state_mav(const mavros_msgs::State::ConstPtr& msg_in);
		void callback_state_battery(const sensor_msgs::BatteryState::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
};
};
