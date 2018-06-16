#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <controller_aug/controller_aug_params.h>

ControllerAugParams::ControllerAugParams(ros::NodeHandle *nh, ros::NodeHandle *nhp) :
	nh_(nh),
	nhp_(nhp),
	pwm_min(1000),
	pwm_max(2000),
	motor_num(0),
	motor_kv(0.0),
	rpm_thrust_m(0.0),
	rpm_thrust_c(0.0),
	motor_drag_max(0.0),
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
	I2z(0.0) {
}

ControllerAugParams::~ControllerAugParams() {
}

void ControllerAugParams::load( void ) {
	ROS_INFO("--== Loading ControllerAug Parameters ==--");

	nh_->param("motor/num", motor_num, motor_num);
	nh_->param("motor/arm_len", la, la);
	nh_->param("motor/kv", motor_kv, motor_kv);
	nh_->param("motor/rpm_thrust_curve/m", rpm_thrust_m, rpm_thrust_m);
	nh_->param("motor/rpm_thrust_curve/c", rpm_thrust_c, rpm_thrust_c);
	nh_->param("motor/drag_max", motor_drag_max, motor_drag_max);

	nh_->param("pwm/min", pwm_min, pwm_min);
	nh_->param("pwm/max", pwm_max, pwm_max);

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

	ROS_INFO("motor:");
	ROS_INFO("  num: %i", motor_num);
	ROS_INFO("  len: %0.4f", la);
	ROS_INFO("  kv: %0.4f", motor_kv);
	ROS_INFO("  T = %0.4fxRPM + %0.4f", rpm_thrust_m, rpm_thrust_c);
	ROS_INFO("  Dmax = %0.4f", motor_drag_max);

	ROS_INFO("pwm: [%i, %i]", pwm_min, pwm_max);

	ROS_INFO("takeoff: [%0.4f, %0.4f, %0.4f]", takeoff_x, takeoff_y, takeoff_z);

	ROS_INFO("bodies: %i", body_num);
	ROS_INFO("  b0:");
	ROS_INFO("    mass: %0.4f", m0);
	ROS_INFO("    Ix: %0.4f", I0x);
	ROS_INFO("    Iy: %0.4f", I0y);
	ROS_INFO("    Iz: %0.4f", I0z);
	ROS_INFO("    com: %0.4f", 0.0);
	ROS_INFO("  b1:");
	ROS_INFO("    mass: %0.4f", m1);
	ROS_INFO("    Ix: %0.4f", I1x);
	ROS_INFO("    Iy: %0.4f", I1y);
	ROS_INFO("    Iz: %0.4f", I1z);
	ROS_INFO("    com: %0.4f", lc1);
	ROS_INFO("  b2:");
	ROS_INFO("    mass: %0.4f", m2);
	ROS_INFO("    Ix: %0.4f", I2x);
	ROS_INFO("    Iy: %0.4f", I2y);
	ROS_INFO("    Iz: %0.4f", I2z);
	ROS_INFO("    com: %0.4f", lc2);

	ROS_INFO("--== ControllerAug Parameters Loaded ==--");
}

