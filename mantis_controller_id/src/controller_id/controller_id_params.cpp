#include <ros/ros.h>

#include <controller_id/controller_id_params.h>

ControllerIDParams::ControllerIDParams(ros::NodeHandle *nh) :
	nh_(nh),
	pwm_min(1000),
	pwm_max(2000),
	num_motors(4),
	gain_ang_roll_p(0.0),
	gain_ang_pitch_p(0.0),
	gain_ang_yaw_p(0.0),
	gain_ang_r1_p(0.0),
	gain_ang_r2_p(0.0),
	gain_rate_roll_p(0.0),
	gain_rate_pitch_p(0.0),
	gain_rate_yaw_p(0.0),
	gain_rate_r1_p(0.0),
	gain_rate_r2_p(0.0),
	la(0.0),
	l0(0.0),
	l1(0.0),
	l2(0.0),
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
	I2z(0.0) {
}

ControllerIDParams::~ControllerIDParams() {
}

void ControllerIDParams::load( void ) {
	nh_->param("pwm/min", pwm_min, pwm_min);
	nh_->param("pwm/max", pwm_max, pwm_max);
	nh_->param("num_motors", num_motors, num_motors);
	nh_->param("gain/ang/roll/p", gain_ang_roll_p, gain_ang_roll_p);
	nh_->param("gain/ang/pitch/p", gain_ang_pitch_p, gain_ang_pitch_p);
	nh_->param("gain/ang/yaw/p", gain_ang_yaw_p, gain_ang_yaw_p);
	nh_->param("gain/ang/r1/p", gain_ang_r1_p, gain_ang_r1_p);
	nh_->param("gain/ang/r2/p", gain_ang_r2_p, gain_ang_r2_p);
	nh_->param("gain/rate/roll/p", gain_rate_roll_p, gain_rate_roll_p);
	nh_->param("gain/rate/pitch/p", gain_rate_pitch_p, gain_rate_pitch_p);
	nh_->param("gain/rate/yaw/p", gain_rate_yaw_p, gain_rate_yaw_p);
	nh_->param("gain/rate/r1/p", gain_rate_r1_p, gain_rate_r1_p);
	nh_->param("gain/rate/r2/p", gain_rate_r2_p, gain_rate_r2_p);

	nh_->param("length/la", la, la);
	nh_->param("length/l0", l0, l0);
	nh_->param("length/l1", l1, l1);
	nh_->param("length/l2", l2, l2);
	nh_->param("length/lc1", lc1, lc1);
	nh_->param("length/lc2", lc2, lc2);

	nh_->param("mass/b0/m", m0, m0);
	nh_->param("mass/b0/Ix", I0x, I0x);
	nh_->param("mass/b0/Iy", I0y, I0y);
	nh_->param("mass/b0/Iz", I0z, I0z);
	nh_->param("mass/b1/m", m1, m1);
	nh_->param("mass/b1/Ix", I1x, I1x);
	nh_->param("mass/b1/Iy", I1y, I1y);
	nh_->param("mass/b1/Iz", I1z, I1z);
	nh_->param("mass/b2/m", m2, m2);
	nh_->param("mass/b2/Ix", I2x, I2x);
	nh_->param("mass/b2/Iy", I2y, I2y);
	nh_->param("mass/b2/Iz", I2z, I2z);

	ROS_INFO("ControllerID params loaded!");
}
