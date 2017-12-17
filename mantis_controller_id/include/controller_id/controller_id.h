#pragma once

#include <ros/ros.h>

//#include <mantis_controller_id/GoalPose.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include <eigen3/Eigen/Dense>
#include <string>

class ControllerID {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_joints_;
		ros::Subscriber sub_goal_accel_;
		ros::Subscriber sub_goal_joints_;

		ros::Publisher pub_rc_;
		ros::Publisher pub_r1_;
		ros::Publisher pub_r2_;
		ros::Publisher pub_twist_;
		ros::Publisher pub_accel_;

		nav_msgs::Odometry msg_state_odom_;
		sensor_msgs::JointState msg_state_joints_;
		geometry_msgs::AccelStamped msg_goal_accel_;
		sensor_msgs::JointState msg_goal_joints_;
		//mantis_controller_id::GoalPose msg_goal_;

		std::string param_model_name_;
		double param_rate_;
		uint16_t param_pwm_min_;
		uint16_t param_pwm_max_;

	public:
		ControllerID( void );

		~ControllerID( void );

		Eigen::Vector3d rot2euler(Eigen::Matrix3d r);
		int16_t map_pwm(double val);
		void goal_rates(void);

		void callback_control(const ros::TimerEvent& e);
		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
		void callback_goal_accel(const geometry_msgs::AccelStamped::ConstPtr& msg_in);
		void callback_goal_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
};
