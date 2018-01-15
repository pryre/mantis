#include <ros/ros.h>

#include <controller_id/controller_id_params.h>

ControllerIDParams::ControllerIDParams(ros::NodeHandle *nh) :
	nh_(nh),
	pwm_min(1000),
	pwm_max(2000),
	motor_num(4),
	motor_thrust_max(0.0),
	motor_drag_max(0.0),
	link_num(2),
	link_servo_torque_max(0.0),
	link_servo_ang_max(0.0),
	vel_max(0.0),
	takeoff_x(0.0),
	takeoff_y(0.0),
	takeoff_z(0.0),
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

	nh_->param("motor/num", motor_num, motor_num);
	nh_->param("motor/thrust_max", motor_thrust_max, motor_thrust_max);
	nh_->param("motor/drag_max", motor_drag_max, motor_drag_max);

	nh_->param("link/num", link_num, link_num);
	nh_->param("link/servo/torque_max", link_servo_torque_max, link_servo_torque_max);
	nh_->param("link/servo/ang_max", link_servo_ang_max, link_servo_ang_max);

	nh_->getParam("gain/position", gain_position);
	nh_->getParam("gain/rotation", gain_rotation);
	nh_->getParam("gain/manipulator", gain_manipulator);

	nh_->param("vel_max", vel_max, vel_max);

	nh_->param("takeoff/x", takeoff_x, takeoff_x);
	nh_->param("takeoff/y", takeoff_y, takeoff_y);
	nh_->param("takeoff/z", takeoff_z, takeoff_z);

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
