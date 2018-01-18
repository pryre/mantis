#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

class ControllerIDParams {
	private:
		ros::NodeHandle *nh_;

	public:
		int pwm_min;
		int pwm_max;

		int motor_num;
		double motor_thrust_max;
		double motor_drag_max;

		double servo_torque_max;
		double servo_ang_max;

		int link_num;

		std::vector<double> gain_position;
		std::vector<double> gain_rotation;
		std::vector<double> gain_manipulator;

		double vel_max;

		double takeoff_x;
		double takeoff_y;
		double takeoff_z;

		int body_num;
		int manip_num;
		double la;
		double lc1;
		double lc2;
		double m0;
		double m1;
		double m2;
		double I0x;
		double I0y;
		double I0z;
		double I1x;
		double I1y;
		double I1z;
		double I2x;
		double I2y;
		double I2z;

	public:
		ControllerIDParams( ros::NodeHandle *nh );

		~ControllerIDParams( void );

		void load( void );
};
