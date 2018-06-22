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

#include <mantis_description/param_client.h>

#include <eigen3/Eigen/Dense>

class MantisStateServer {
	private:
		ros::NodeHandle *nh_;
		ros::NodeHandle *nhp_;
		ros::Timer timer_estimator_;

		ros::Subscriber sub_state_imu_;
		ros::Subscriber sub_state_mav_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_battery_;
		ros::Subscriber sub_state_joints_;

		ros::Publisher pub_state_;

		MantisParamClient p_;

		bool param_use_odom_avel_;
		double param_rate_;
		ros::Time msg_odom_tr_;
		ros::Time msg_battery_tr_;
		ros::Time msg_joints_tr_;
		ros::Time msg_imu_tr_;
		ros::Time msg_mav_state_tr_;

		ros::Time time_last_est_;

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
		MantisStateServer( ros::NodeHandle *nh, ros::NodeHandle *nhp );

		~MantisStateServer( void );

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

		//Conversion Helpers
		Eigen::Vector3d vector_from_msg(const geometry_msgs::Vector3 v);
		Eigen::Vector3d point_from_msg(const geometry_msgs::Point p);
		Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);

		geometry_msgs::Vector3 vector_from_eig(const Eigen::Vector3d &v);
		geometry_msgs::Point point_from_eig(const Eigen::Vector3d &p);
		geometry_msgs::Quaternion quaternion_from_eig(const Eigen::Quaterniond &q);
		geometry_msgs::Pose pose_from_eig(const Eigen::Affine3d &g);
};
