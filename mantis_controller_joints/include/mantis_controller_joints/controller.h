#pragma once

#include <ros/ros.h>

#include <pid_controller_lib/pidController.h>

#include <std_msgs/Float64.h>

#include <string>

class Controller {
	private:
		ros::NodeHandle *nhp_;
		ros::Publisher pub_output_;
		ros::Subscriber sub_reference_;

		pidController pid_;

		std::string name_;

		double state_position_;
		double state_velocity_;

		bool have_reference_;

		double output_;
		double ref_;

	public:
		Controller( ros::NodeHandle *nhp, std::string joint_name );

		~Controller( void );

		void callback_reference( const std_msgs::Float64::ConstPtr& msg_in );

		void set_state(double position, double velocity );

		void do_control( double dt );

		double output( void );
		double reference( void );
		std::string name( void );
};
