#include <ros/ros.h>

#include <controller_id/controller_id.h>
#include <controller_id/calc_Dq.h>
#include <controller_id/calc_Cqqd.h>
#include <controller_id/calc_Lqd.h>
#include <controller_id/calc_Nq.h>
#include <controller_id/calc_G0.h>
#include <controller_id/calc_G1.h>
#include <controller_id/calc_G2.h>

#include <controller_id/GoalPose.h>
#include <mavros_msgs/RCOut.h>
#include <nav_msgs/Odometry.h>

#include <Eigen3/Dense>

//TODO: use params
#define NUM_MOTORS 6

ControllerID::ControllerID() :
	nh_("~"),
	param_model_name_("mantis_uav"),
	param_pwm_min_(1000),
	param_pwm_max_(2000) {

	pub_rc_ = nh_.advertise<mavros_msgs::RCOut>("rc", 10);

	sub_odom_ = nh_.subscribe<nav_msgs::Odometry>( "odom", 10, &ControllerID::callback_odom, this );
	sub_goal_ = nh_.subscribe<controller_id::GoalPose>( "goal", 10, &ControllerID::callback_goal, this );

	timer_ = nh_.createTimer(ros::Duration(1.0), &ControllerID::callback_control, this );
}

ControllerID::~ControllerID() {
}

void ControllerID::callback_control(const ros::TimerEvent& e) {
	mavros_msgs::RCOut msg_rc_out;
	msg_rc_out.header.stamp = event.current_real;
	msg_rc_out.header.frame_id = "map";
	msg_rc_out.channels.reserve(NUM_MOTORS);	//Allocate space for the number of motors


	if( ( msg_odom_.header.stamp != ros::Time(0) ) &&
		( msg_goal_.header.stamp != ros::Time(0) ) ) {

		Eigen::MatrixXd q(18,1);	//g(4,4) + r(2,1)
		Eigen::MatrixXd qd(8,1);	//v(6,1) + rd(2,1)
		Eigen::MatrixXd ua(8,1);	//vd(6,1) + rdd(2,1)
		Eigen::MatrixXd tau(8,1);	//base torque, base force, arm torque
		Eigen::MatrixXd u(8,1);		//Motor outputs (6), arm links

		Eigen::MatrixXd Dq(8,8);
		Eigen::MatrixXd Cqqd(8,8);
		Eigen::MatrixXd Lqd(8,8);
		Eigen::MatrixXd Nq(8,1);

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

void ControllerID::callback_goal(const controller_id::GoalPose::ConstPtr& msg_in) {
	msg_goal_ = *msg_in;
}
