#include <ros/ros.h>

#include <controller_id/controller_id.h>
#include <controller_id/calc_Dq.h>
#include <controller_id/calc_Cqqd.h>
#include <controller_id/calc_Lqd.h>
#include <controller_id/calc_Nq.h>
#include <controller_id/calc_G0.h>
#include <controller_id/calc_G1.h>
#include <controller_id/calc_G2.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <mantis_controller_id/GoalPose.h>

#include <mavros_msgs/RCOut.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

//TODO: use params
#define NUM_MOTORS 6

ControllerID::ControllerID() :
	nh_("~"),
	param_model_name_("mantis_uav"),
	param_rate_(100),
	param_pwm_min_(1000),
	param_pwm_max_(2000) {

	pub_rc_ = nh_.advertise<mavros_msgs::RCOut>("rc", 10);
	pub_r1_ = nh_.advertise<std_msgs::Float64>("r1", 10);
	pub_r2_ = nh_.advertise<std_msgs::Float64>("r2", 10);

	sub_odom_ = nh_.subscribe<nav_msgs::Odometry>( "odom", 10, &ControllerID::callback_odom, this );
	sub_joints_ = nh_.subscribe<sensor_msgs::JointState>( "joint_state", 10, &ControllerID::callback_joints, this );
	sub_goal_ = nh_.subscribe<mantis_controller_id::GoalPose>( "goal", 10, &ControllerID::callback_goal, this );

	timer_ = nh_.createTimer(ros::Duration(1.0/param_rate_), &ControllerID::callback_control, this );
}

ControllerID::~ControllerID() {
}

void ControllerID::callback_control(const ros::TimerEvent& e) {
	mavros_msgs::RCOut msg_rc_out;
	std_msgs::Float64 msg_r1_out;
	std_msgs::Float64 msg_r2_out;

	msg_rc_out.header.stamp = e.current_real;
	msg_rc_out.header.frame_id = "map";
	msg_rc_out.channels.resize(NUM_MOTORS);	//Allocate space for the number of motors

	//TODO: LOAD THROUGH PARAMS
	double la = 0.275;
	double l1 = 0.2;
	double l2 = 0.2;
	double m0 = 2.0;
	double m1 = 0.005;
	double m2 = 0.2;
	double g = 9.80665;
	double kt = 1.0/(NUM_MOTORS*0.7*g);
	double km = 0.5;
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
		( msg_joints_.header.stamp != ros::Time(0) ) ) { //TODO: &&
		//TODO: ( msg_goal_.header.stamp != ros::Time(0) ) ) {

		ROS_INFO_ONCE("Inverse dynamics controller running!");

		Eigen::MatrixXd q = Eigen::MatrixXd::Constant(18, 1, 0.0);	//g(4,4) + r(2,1)
		Eigen::MatrixXd qd = Eigen::MatrixXd::Constant(8, 1, 0.0);	//v(6,1) + rd(2,1)
		Eigen::MatrixXd ua = Eigen::MatrixXd::Constant(8, 1, 0.0);	//vd(6,1) + rdd(2,1)
		Eigen::MatrixXd tau = Eigen::MatrixXd::Constant(8, 1, 0.0);	//base torque, base force, arm torque

		Eigen::MatrixXd D = Eigen::MatrixXd::Constant(8, 8, 0.0);
		Eigen::MatrixXd C = Eigen::MatrixXd::Constant(8, 8, 0.0);
		Eigen::MatrixXd L = Eigen::MatrixXd::Constant(8, 8, 0.0);
		Eigen::MatrixXd N = Eigen::MatrixXd::Constant(8, 1, 0.0);

		//XXX: Quick init for states
		Eigen::Quaterniond gq;
		gq.x() = msg_odom_.pose.pose.orientation.x;
		gq.y() = msg_odom_.pose.pose.orientation.y;
		gq.z() = msg_odom_.pose.pose.orientation.z;
		gq.w() = msg_odom_.pose.pose.orientation.w;
		Eigen::Matrix3d gr = gq.normalized().toRotationMatrix();
		double r1 = msg_joints_.position[0];
		double r2 = msg_joints_.position[1];

		double bvx = msg_odom_.twist.twist.linear.x;
		double bvy = msg_odom_.twist.twist.linear.y;
		double bvz = msg_odom_.twist.twist.linear.z;
		double bwx = msg_odom_.twist.twist.angular.x;
		double bwy = msg_odom_.twist.twist.angular.y;
		double bwz = msg_odom_.twist.twist.angular.z;
		double r1d = msg_joints_.velocity[0];
		double r2d = msg_joints_.velocity[1];

		//XXX: Zero velocity for now
		ua(0,0) = 0.2*(0.0 - bwx);
		ua(1,0) = 0.2*(0.0 - bwy);
		ua(2,0) = 0.2*(0.0 - bwz);
		ua(3,0) = 0.2*(0.0 - bvx);
		ua(4,0) = 0.2*(0.0 - bvy);
		ua(5,0) = 0.2*(0.0 - bvz);
		ua(6,0) = 0.2*(0.0 - r1d);
		ua(7,0) = 0.2*(0.0 - r2d);

		calc_Dq(D, IJ0x, IJ0y, IJ0z, IJ1x, IJ1y, IJ1z, IJ2x, IJ2y, IJ2z, l1, l2, m0, m1, m2, r1, r2);
		calc_Cqqd(C, IJ1x, IJ1y, IJ1z, IJ2x, IJ2y, IJ2z, bvx, bvy, bvz, bwx, bwy, bwz, l1, l2, r1, r1d, r2, r2d);
		calc_Lqd(L);
		calc_Nq(N, IJ1y, IJ1z, IJ2x, IJ2y, IJ2z, g, gr(2,0), gr(2,1), gr(2,2), l1, l2, m0, m1, m2, r1, r2);

		tau = D*ua + (C + L)*qd + N;

		//XXX: Hardcoded for hex because lazy
		Eigen::MatrixXd M = Eigen::MatrixXd::Constant(8, 8, 0.0);

		double mr30 = std::cos(M_PI/6.0);
		double mr60 = std::cos(M_PI/3.0);
		M <<      -la*kt,           0,  km, 0, 0, kt, 0, 0,
			       la*kt,           0, -km, 0, 0, kt, 0, 0,
			  mr30*la*kt,  mr60*la*kt,  km, 0, 0, kt, 0, 0,
			 -mr30*la*kt, -mr60*la*kt, -km, 0, 0, kt, 0, 0,
			 -mr30*la*kt,  mr60*la*kt, -km, 0, 0, kt, 0, 0,
			  mr30*la*kt, -mr60*la*kt,  km, 0, 0, kt, 0, 0,
			           0,           0,   0, 0, 0,  0, 1, 0,
			  	       0,           0,   0, 0, 0,  0, 0, 1;
		//XXX: Hardcoded for hex because lazy

		Eigen::MatrixXd u = M*tau;

		for(int i=0; i<NUM_MOTORS; i++) {
			msg_rc_out.channels[i] = map_pwm(u(i,0));
		}

		msg_r1_out.data = u(NUM_MOTORS,0);
		msg_r2_out.data = u(NUM_MOTORS+1,0);
	} else {
		//Output nothing until the input info is available
		for(int i=0; i<NUM_MOTORS; i++) {
			msg_rc_out.channels[i] = param_pwm_min_;
		}

		msg_r1_out.data = 0.0;
		msg_r2_out.data = 0.0;
	}

	pub_rc_.publish(msg_rc_out);
	pub_r1_.publish(msg_r1_out);
	pub_r2_.publish(msg_r2_out);
}

int16_t ControllerID::map_pwm(double val) {
	//Constrain from 0 -> 1
	double c = (val > 1.0) ? 1.0 : (val < 0.0) ? 0.0 : val;

	//Scale c to the pwm values
	return int16_t((param_pwm_max_ - param_pwm_min_)*c) + param_pwm_min_;
}

void ControllerID::callback_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	msg_odom_ = *msg_in;
}

void ControllerID::callback_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	msg_joints_ = *msg_in;
}

void ControllerID::callback_goal(const mantis_controller_id::GoalPose::ConstPtr& msg_in) {
	msg_goal_ = *msg_in;
}
