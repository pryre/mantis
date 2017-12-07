#include <ros/ros.h>

#include <controller_id/controller_id.h>
#include <controller_id/calc_Dq.h>
#include <controller_id/calc_Cqqd.h>
#include <controller_id/calc_Lqd.h>
#include <controller_id/calc_Nq.h>
#include <controller_id/calc_G0.h>
#include <controller_id/calc_G1.h>
#include <controller_id/calc_G2.h>

#include <mantis_controller_id/GoalPose.h>
#include <mavros_msgs/RCOut.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

//TODO: use params
#define NUM_MOTORS 6

ControllerID::ControllerID() :
	nh_("~"),
	param_model_name_("mantis_uav"),
	param_pwm_min_(1000),
	param_pwm_max_(2000) {

	pub_rc_ = nh_.advertise<mavros_msgs::RCOut>("rc", 10);

	sub_odom_ = nh_.subscribe<nav_msgs::Odometry>( "odom", 10, &ControllerID::callback_odom, this );
	sub_goal_ = nh_.subscribe<mantis_controller_id::GoalPose>( "goal", 10, &ControllerID::callback_goal, this );

	timer_ = nh_.createTimer(ros::Duration(1.0), &ControllerID::callback_control, this );
}

ControllerID::~ControllerID() {
}

void ControllerID::callback_control(const ros::TimerEvent& e) {
	mavros_msgs::RCOut msg_rc_out;
	msg_rc_out.header.stamp = e.current_real;
	msg_rc_out.header.frame_id = "map";
	msg_rc_out.channels.reserve(NUM_MOTORS);	//Allocate space for the number of motors


	//TODO: LOAD THROUGH PARAMS
	double la = 0.275;
	double l1 = 0.2;
	double l2 = 0.2;
	double m0 = 2.0;
	double m1 = 0.005;
	double m2 = 0.2;
	double g = 9.80665;
	double IJ0x = (0.05*0.05 + la*la)/12;
	double IJ0y = (0.05*0.05 + la*la)/12;
	double IJ0z = la*la/2;
	double IJ1x = (0.05*0.05 + l1*l1)/12;
	double IJ1y = (0.05*0.05 + l1*l1)/12;
	double IJ1z = (2*0.05*0.05)/12;
	double IJ2x = (0.05*0.05 + l2*l2)/12;
	double IJ2y = (0.05*0.05 + l2*l2)/12;
	double IJ2z = (2*0.05*0.05)/12;
	//TODO: LOAD THROUGH PARAMS


	if( ( msg_odom_.header.stamp != ros::Time(0) ) &&
		( msg_goal_.header.stamp != ros::Time(0) ) ) {

		Eigen::MatrixXd q(18,1);	//g(4,4) + r(2,1)
		Eigen::MatrixXd qd(8,1);	//v(6,1) + rd(2,1)
		Eigen::MatrixXd ua(8,1);	//vd(6,1) + rdd(2,1)
		Eigen::MatrixXd tau(8,1);	//base torque, base force, arm torque
		Eigen::MatrixXd u(8,1);		//Motor outputs (6), arm links

		Eigen::MatrixXd D(8,8);
		Eigen::MatrixXd C(8,8);
		Eigen::MatrixXd L(8,8);
		Eigen::MatrixXd N(8,1);

		calc_Dq(D, IJ1x, IJ1y, IJ1z, IJ2x, IJ2y, IJ2z, IJ0x, IJ0y, IJ0z, l1, l2, m0, m1, m2, q(16,1), q(17,1));

		tau = D*ua + (C + L)*qd + N;
		//u = M*tau;
	} else {
		//Output nothing until the input info is available
		for(int i=0; i<NUM_MOTORS; i++) {
			msg_rc_out.channels[i] = param_pwm_min_;
		}
	}

	pub_rc_.publish(msg_rc_out);
}

void ControllerID::callback_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	msg_odom_ = *msg_in;
}

void ControllerID::callback_goal(const mantis_controller_id::GoalPose::ConstPtr& msg_in) {
	msg_goal_ = *msg_in;
}
