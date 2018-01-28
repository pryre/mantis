#pragma once

#include <ros/ros.h>

#include <pidController/pidController.h>

#include <std_msgs/Float64.h>

#include <string>

class Controller {
	private:
		ros::Publisher pub_output_;
		ros::Subscriber sub_reference_;

		pidController pid_;

		std::string name_;

		double state_position_;
		double state_velocity_;

		bool have_reference_;

		double param_limit_min_;
		double param_limit_max_;
		double param_gain_p_;
		double param_gain_i_;
		double param_gain_d_;
		double output_;
		double ref_;

	public:
		Controller( void );

		~Controller( void );

		bool init( ros::NodeHandle *nh, std::string controller_name, std::string joint_name );

		void callback_reference( const std_msgs::Float64::ConstPtr& msg_in );

		void set_state(double position, double velocity );

		void do_control( double dt );

		double output( void );
		double reference( void );
		std::string name( void );

};
