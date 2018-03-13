#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <controller_id/controller_id_params.h>

ControllerIDParams::ControllerIDParams(ros::NodeHandle *nh, ros::NodeHandle *nhp) :
	nh_(nh),
	nhp_(nhp),
	pwm_min(1000),
	pwm_max(2000),
	motor_num(0),
	motor_thrust_max(0.0),
	motor_drag_max(0.0),
	link_num(0),
	servo_torque_max(0.0),
	servo_ang_max(0.0),
	vel_max(0.0),
	takeoff_x(0.0),
	takeoff_y(0.0),
	takeoff_z(0.0),
	la(0.0),
	body_num(1),
	manip_num(0),
	lc1(0.0),
	lc2(0.0),
	m0(0.0),
	m1(0.0),
	m2(0.0),
	I0x(0.0),
	I0y(0.0),
	I0z(0.0),
	I1x(0.0),
	I1y(0.0),
	I1z(0.0),
	I2x(0.0),
	I2y(0.0),
	I2z(0.0),
	dyncfg_gain_pos_xy_(ros::NodeHandle(*nhp, "gain/pos/xy")),
	dyncfg_gain_pos_z_(ros::NodeHandle(*nhp, "gain/pos/z")),
	dyncfg_gain_vel_xy_(ros::NodeHandle(*nhp, "gain/vel/xy")),
	dyncfg_gain_vel_z_(ros::NodeHandle(*nhp, "gain/vel/z")),
	dyncfg_gain_rot_ang_(ros::NodeHandle(*nhp, "gain/rot/ang")),
	dyncfg_gain_rot_rate_(ros::NodeHandle(*nhp, "gain/rot/rate")) {

	dyncfg_gain_pos_xy_.setCallback(boost::bind(&ControllerIDParams::callback_cfg_gain_pos_xy, this, _1, _2));
	dyncfg_gain_vel_xy_.setCallback(boost::bind(&ControllerIDParams::callback_cfg_gain_vel_xy, this, _1, _2));
	dyncfg_gain_pos_z_.setCallback(boost::bind(&ControllerIDParams::callback_cfg_gain_pos_z, this, _1, _2));
	dyncfg_gain_vel_z_.setCallback(boost::bind(&ControllerIDParams::callback_cfg_gain_vel_z, this, _1, _2));
	dyncfg_gain_rot_ang_.setCallback(boost::bind(&ControllerIDParams::callback_cfg_gain_rot_ang, this, _1, _2));
	dyncfg_gain_rot_rate_.setCallback(boost::bind(&ControllerIDParams::callback_cfg_gain_rot_rate, this, _1, _2));
}

ControllerIDParams::~ControllerIDParams() {
}

void ControllerIDParams::load( void ) {
	nh_->param("pwm/min", pwm_min, pwm_min);
	nh_->param("pwm/max", pwm_max, pwm_max);

	nh_->param("motor/num", motor_num, motor_num);
	nh_->param("motor/arm_len", la, la);
	nh_->param("motor/thrust_max", motor_thrust_max, motor_thrust_max);
	nh_->param("motor/drag_max", motor_drag_max, motor_drag_max);

	nh_->param("servo/torque_max", servo_torque_max, servo_torque_max);
	nh_->param("servo/ang_max", servo_ang_max, servo_ang_max);

	nh_->param("links/num", link_num, link_num);

	nh_->param("vel_max", vel_max, vel_max);

	nh_->param("takeoff/x", takeoff_x, takeoff_x);
	nh_->param("takeoff/y", takeoff_y, takeoff_y);
	nh_->param("takeoff/z", takeoff_z, takeoff_z);

	nh_->param("body/num", body_num, body_num);
	manip_num = body_num - 1;
	nh_->param("body/b0/mass/m", m0, m0);
	nh_->param("body/b0/mass/Ix", I0x, I0x);
	nh_->param("body/b0/mass/Iy", I0y, I0y);
	nh_->param("body/b0/mass/Iz", I0z, I0z);
	nh_->param("body/b1/mass/m", m1, m1);
	nh_->param("body/b1/mass/Ix", I1x, I1x);
	nh_->param("body/b1/mass/Iy", I1y, I1y);
	nh_->param("body/b1/mass/Iz", I1z, I1z);
	nh_->param("body/b1/com", lc1, lc1);
	nh_->param("body/b2/mass/m", m2, m2);
	nh_->param("body/b2/mass/Ix", I2x, I2x);
	nh_->param("body/b2/mass/Iy", I2y, I2y);
	nh_->param("body/b2/mass/Iz", I2z, I2z);
	nh_->param("body/b2/com", lc2, lc2);

	ROS_INFO("ControllerID params loaded!");
}

void ControllerIDParams::callback_cfg_gain_pos_xy(mantis_controller_id::PIGainsConfig &config, uint32_t level) {
	gain_position_xy_p = config.p;
	gain_position_xy_i = config.i;
}


void ControllerIDParams::callback_cfg_gain_pos_z(mantis_controller_id::PIGainsConfig &config, uint32_t level) {
	gain_position_z_p = config.p;
	gain_position_z_i = config.i;

}

void ControllerIDParams::callback_cfg_gain_vel_xy(mantis_controller_id::PGainsConfig &config, uint32_t level) {
	gain_velocity_xy_p = config.p;
}

void ControllerIDParams::callback_cfg_gain_vel_z(mantis_controller_id::PGainsConfig &config, uint32_t level) {
	gain_velocity_z_p = config.p;

}

void ControllerIDParams::callback_cfg_gain_rot_ang(mantis_controller_id::PGainsConfig &config, uint32_t level) {
	gain_rotation_ang_p = config.p;
}

void ControllerIDParams::callback_cfg_gain_rot_rate(mantis_controller_id::PGainsConfig &config, uint32_t level) {
	gain_rotation_rate_p = config.p;
}

