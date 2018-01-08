#pragma once

#include <ros/ros.h>

#include <string>

class ControllerIDParams {
	private:
		ros::NodeHandle *nh_;

	public:
		int pwm_min;
		int pwm_max;

		int motor_num;
		double motor_thrust_max;
		double motor_drag_max;

		int link_num;
		double link_servo_torque_max;
		double link_servo_ang_max;

		double gain_ang_roll_p;
		double gain_ang_pitch_p;
		double gain_ang_yaw_p;
		double gain_ang_r1_p;
		double gain_ang_r2_p;
		double gain_rate_roll_p;
		double gain_rate_pitch_p;
		double gain_rate_yaw_p;
		double gain_rate_r1_p;
		double gain_rate_r2_p;

		double la;
		double l0;
		double l1;
		double l2;
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
