#pragma once

#include <ros/ros.h>

#include <controller_id/GoalPose.h>
#include <nav_msgs/Odometry.h>

class ControllerID {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Publisher pub_rc_;
		ros::Subscriber sub_odom_;
		ros::Subscriber sub_goal_;

		nav_msgs::Odometry msg_odom_;
		controller_id::GoalPose msg_goal_pose_;

		param_model_name_;
		param_pwm_min_;
		param_pwm_max_;

	public:
		ControllerID( void );

		~ControllerID( void );

		void callback_control(const ros::TimerEvent& e);
		void callback_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_goal(const controller_id::GoalPose::ConstPtr& msg_in);
};
