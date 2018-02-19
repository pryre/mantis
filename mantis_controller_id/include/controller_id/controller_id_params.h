#pragma once

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <mantis_controller_id/PDGainsConfig.h>

#include <string>
#include <vector>

class ControllerIDParams {
	private:
		ros::NodeHandle *nh_;
		ros::NodeHandle *nhp_;

	public:
		int pwm_min;
		int pwm_max;

		int motor_num;
		double motor_thrust_max;
		double motor_drag_max;

		double servo_torque_max;
		double servo_ang_max;

		int link_num;

		double gain_position_xy_p;
		double gain_position_xy_d;
		double gain_position_z_p;
		double gain_position_z_d;
		double gain_rotation_p;
		double gain_rotation_d;

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

		dynamic_reconfigure::Server<mantis_controller_id::PDGainsConfig> dyncfg_gain_xy_;
		dynamic_reconfigure::Server<mantis_controller_id::PDGainsConfig> dyncfg_gain_z_;
		dynamic_reconfigure::Server<mantis_controller_id::PDGainsConfig> dyncfg_gain_rot_;

	public:
		ControllerIDParams( ros::NodeHandle *nh, ros::NodeHandle *nhp );

		~ControllerIDParams( void );

		void load( void );

	private:
		void callback_cfg_gain_xy(mantis_controller_id::PDGainsConfig &config, uint32_t level);
		void callback_cfg_gain_z(mantis_controller_id::PDGainsConfig &config, uint32_t level);
		void callback_cfg_gain_rot(mantis_controller_id::PDGainsConfig &config, uint32_t level);
};
