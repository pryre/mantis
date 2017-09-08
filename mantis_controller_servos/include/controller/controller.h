#pragma once

#include <ros/ros.h>

#include <pidController/pidController.h>

#include <std_msgs/Float64.h>

#include <vector>
#include <string>

class Controller {
	private:
		ros::Publisher pub_goal_;
		ros::Subscriber sub_sp_;

		double state_angle_;
		double state_velocity_;
		double state_effort_;

		double sp_angle_;
		double sp_velocity_;
		double sp_effort_;

		double param_ang_ff_;
		double param_vel_kp_;
		double param_vel_ki_;
		double param_vel_kd_;
		double param_vel_tau_;
		double param_vel_max_;
		double param_effort_max_;

		pidController controller_velocity_;

		std_msgs::Float64 msg_out_;
		bool have_setpoint_;

	public:
		Controller( void );

		~Controller( void );

		void init( ros::NodeHandle *nh, std::string model_name, std::string joint_name );

		void callback_setpoint( const std_msgs::Float64::ConstPtr& msg_in );

		void set_states(double ref_angle, double ref_velocity, double ref_effort);

		void do_control( double dt );

		double get_goal_angle( void );
		double get_goal_velocity( void );
		double get_goal_effort( void );

		double double_constrain(const double x, const double min, const double max);
};
