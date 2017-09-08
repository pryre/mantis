#pragma once

#include <ros/ros.h>

#include <controller/controller.h>

#include <sensor_msgs/JointState.h>
#include <vector>

class Spawner {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Subscriber sub_state_;
		ros::Publisher pub_goal_;

		std::string param_model_name_;

		std::vector<Controller> controllers;

		double param_servo_update_rate_;

		sensor_msgs::JointState servo_states_;
		sensor_msgs::JointState servo_goals_;
		bool got_states_;

	public:
		Spawner( void );

		~Spawner( void );

		void callback_timer(const ros::TimerEvent& e);
		void callback_state(const sensor_msgs::JointState::ConstPtr& msg_in);
};
