#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

class ControllerAugParams {
	private:
		ros::NodeHandle *nh_;
		ros::NodeHandle *nhp_;

	public:
		int pwm_min;
		int pwm_max;

		double la;
		int motor_num;
		//double motor_thrust_max;
		double motor_kv;
		double rpm_thrust_m;
		double rpm_thrust_c;
		double motor_drag_max;

		int link_num;

		double takeoff_x;
		double takeoff_y;
		double takeoff_z;

		int body_num;
		int manip_num;
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
		ControllerAugParams( ros::NodeHandle *nh, ros::NodeHandle *nhp );

		~ControllerAugParams( void );

		void load( void );

	//private:
};
