#include <ros/ros.h>

#include <controller_id/controller_id.h>
#include <controller_id/controller_id_params.h>
#include <dynamics/calc_Dq.h>
#include <dynamics/calc_Cqqd.h>
#include <dynamics/calc_Lqd.h>
//#include <dynamics/calc_Nq.h>
//#include <dynamics/calc_Mm.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
//#include <mantis_controller_id/GoalPose.h>

#include <mavros_msgs/RCOut.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

ControllerID::ControllerID() :
	nh_("~"),
	p_(&nh_),
	param_model_name_("mantis_uav"),
	param_rate_(100) {

	p_.load();

	pub_rc_ = nh_.advertise<mavros_msgs::RCOut>("rc", 10);
	pub_r1_ = nh_.advertise<std_msgs::Float64>("r1", 10);
	pub_r2_ = nh_.advertise<std_msgs::Float64>("r2", 10);
	pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("feedback/twist", 10);
	pub_accel_ = nh_.advertise<geometry_msgs::AccelStamped>("feedback/accel", 10);
	pub_joints_ = nh_.advertise<sensor_msgs::JointState>("feedback/joints", 10);

	sub_state_odom_ = nh_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &ControllerID::callback_state_odom, this );
	sub_state_joints_ = nh_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &ControllerID::callback_state_joints, this );

	sub_goal_accel_ = nh_.subscribe<geometry_msgs::AccelStamped>( "goal/accel", 10, &ControllerID::callback_goal_accel, this );
	sub_goal_joints_ = nh_.subscribe<sensor_msgs::JointState>( "goal/joints", 10, &ControllerID::callback_goal_joints, this );

	timer_ = nh_.createTimer(ros::Duration(1.0/param_rate_), &ControllerID::callback_control, this );
}

ControllerID::~ControllerID() {
}

void ControllerID::callback_control(const ros::TimerEvent& e) {
	mavros_msgs::RCOut msg_rc_out;
	std_msgs::Float64 msg_r1_out;
	std_msgs::Float64 msg_r2_out;
	geometry_msgs::TwistStamped msg_twist_out;
	geometry_msgs::AccelStamped msg_accel_out;
	sensor_msgs::JointState msg_joints_out;

	msg_rc_out.header.stamp = e.current_real;
	msg_rc_out.header.frame_id = "map";
	msg_rc_out.channels.resize(p_.motor_num);	//Allocate space for the number of motors
	msg_twist_out.header.stamp = e.current_real;
	msg_twist_out.header.frame_id = param_model_name_;
	msg_accel_out.header.stamp = e.current_real;
	msg_accel_out.header.frame_id = param_model_name_;
	msg_joints_out.header.stamp = e.current_real;
	msg_joints_out.header.frame_id = param_model_name_;


	double g = 9.80665;

	/*
	//double frame_height = 0.05;
	//double arm_radius = 0.01;
	//double kt = 1.0/(NUM_MOTORS*0.7*g);
	//double km = 0.5;
	//double IJ0x = m0*(frame_height*frame_height + 3*la*la)/12;
	//double IJ0y = m0*(frame_height*frame_height + 3*la*la)/12;
	//double IJ0z = m0*la*la/2;
	//double IJ1x = m1*(arm_radius*arm_radius)/12;
	//double IJ1y = m1*(3*arm_radius*arm_radius + l1*l1)/12;
	//double IJ1z = m1*(3*arm_radius*arm_radius + l1*l1)/12;
	//double IJ2x = m2*(arm_radius*arm_radius)/12;
	//double IJ2y = m2*(3*arm_radius*arm_radius + l2*l2)/12;
	//double IJ2z = m2*(3*arm_radius*arm_radius + l2*l2)/12;

	double la = 0.275;
	double l0 = -0.05;
	double l1 = 0.1965;
	double l2 = 0.1965;
	double lc1 = 0.1638;
	double lc2 = 0.1638;
	double m0 = 1.63;
	double m1 = 0.09;
	double m2 = 0.09;
	double IJ0x = 0.015399592102914;
	double IJ0y = 0.014552318824581;
	double IJ0z = 0.027781409055000;
	double IJ1x = 0.00001371;
	double IJ1y = 0.00008292;
	double IJ1z = 0.00008075;
	double IJ2x = 0.00001345;
	double IJ2y = 0.00008292;
	double IJ2z = 0.00008075;
	*/

	if( ( msg_state_odom_.header.stamp != ros::Time(0) ) &&
		( msg_state_joints_.header.stamp != ros::Time(0) ) &&
		( msg_goal_accel_.header.stamp != ros::Time(0) ) &&
		( msg_goal_joints_.header.stamp != ros::Time(0) ) ) {

		ROS_INFO_ONCE("Inverse dynamics controller running!");

		double num_states = 6 + p_.link_num;

		Eigen::MatrixXd q = Eigen::MatrixXd::Zero(18, 1);	//g(4,4) + r(2,1)
		Eigen::MatrixXd qd = Eigen::MatrixXd::Zero(num_states, 1);	//v(6,1) + rd(2,1)
		Eigen::MatrixXd ua = Eigen::MatrixXd::Zero(num_states, 1);	//vd(6,1) + rdd(2,1)
		Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(num_states, 1);	//base torque, base force, arm torque

		Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_states, num_states);
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_states, num_states);
		Eigen::MatrixXd L = Eigen::MatrixXd::Zero(num_states, num_states);
		//Eigen::MatrixXd N = Eigen::MatrixXd::Constant(8, 1, 0.0);

		//XXX: Quick init for states
		Eigen::Quaterniond gq;
		gq.x() = msg_state_odom_.pose.pose.orientation.x;
		gq.y() = msg_state_odom_.pose.pose.orientation.y;
		gq.z() = msg_state_odom_.pose.pose.orientation.z;
		gq.w() = msg_state_odom_.pose.pose.orientation.w;
		Eigen::Matrix3d gr = gq.normalized().toRotationMatrix();
		double r1 = msg_state_joints_.position[0];
		double r2 = msg_state_joints_.position[1];

		double bvx = msg_state_odom_.twist.twist.linear.x;
		double bvy = msg_state_odom_.twist.twist.linear.y;
		double bvz = msg_state_odom_.twist.twist.linear.z;
		double bwx = msg_state_odom_.twist.twist.angular.x;
		double bwy = msg_state_odom_.twist.twist.angular.y;
		double bwz = msg_state_odom_.twist.twist.angular.z;
		double r1d = msg_state_joints_.velocity[0];
		double r2d = msg_state_joints_.velocity[1];

		//Do conversions from world to body frame
		//Acceleration vectors for hover
		//TODO: Need to rotate the acceleration vector to match body yaw first

		double ax = msg_goal_accel_.accel.linear.x;
		double ay = msg_goal_accel_.accel.linear.y;
		double az = g + msg_goal_accel_.accel.linear.z;
		az = (az < 0.0) ? 0.0 : az;	//Can't accelerate downward

		Eigen::Vector3d A(ax, ay, az);

		//Start with the goal yaw as the current yaw
		Eigen::Vector3d yaw_c = gr*Eigen::Vector3d::UnitY();
		Eigen::Vector3d sp_x = yaw_c.cross(A).normalized();
		Eigen::Vector3d sp_y = A.cross(sp_x).normalized();
		Eigen::Vector3d sp_z = A.normalized();

		//Build the setpoint rotation matrix
		Eigen::Matrix3d gr_sp;
		gr_sp.col(0) = sp_x;
		gr_sp.col(1) = sp_y;
		gr_sp.col(2) = sp_z;
		gr_sp.normalize();

		//Rotate the accel vector back into the body frame
		//Such that Ab.z will be the amount of acceleration wanted
		Eigen::Vector3d Ab = gr.transpose()*A;

		/* TODO:
		Eigen::Vector3d Ax(ax, 0.0, az);
		Eigen::Vector3d Ay(0.0, ay, az);
		//Eigen::Vector3d Az(0.0, 0.0, az);
		Eigen::Vector3d A(ax, ay, az);
		Eigen::Vector3d Ab = gr*A; //Acceleration in the body frame

		Eigen::Vector3d ucx = gr*Eigen::Vector3d::UnitZ();
		Eigen::Vector3d ucy = gr*Eigen::Vector3d::UnitZ();
		Eigen::Vector3d Cx(ucx(0), 0.0, ucx(2));
		Eigen::Vector3d Cy(0.0, ucy(1), ucy(2));

		//Limit horizontal thrust by z thrust
		double thrust_xy_max = gThrust.getZ() * std::tan( param_tilt_max_ );

		//If thrust_sp_xy_len > thrust_xy_max
		if( xyThrust.length() > thrust_xy_max ) {
			//Scale the XY thrust setpoint down
			double k = thrust_xy_max / xyThrust.length();
			gThrust.setX( k*gThrust.getX() );
			gThrust.setY( k*gThrust.getY() );
			xyThrust.setX( gThrust.getX() );
			xyThrust.setY( gThrust.getY() );
		}

		//Calculate thrust vector angles from goal accels
		double g_phi = std::acos(Eigen::Vector3d::UnitZ().dot(Ay.normalized()));
		double g_theta = std::acos(Eigen::Vector3d::UnitZ().dot(Ax.normalized()));

		double c_phi = std::acos(Eigen::Vector3d::UnitZ().dot(Cy.normalized()));
		double c_theta = std::acos(Eigen::Vector3d::UnitZ().dot(Cx.normalized()));

		//Correct for frame rotations
		g_phi = (ay > 0.0) ? -g_phi : g_phi;
		g_theta = (ax < 0.0) ? -g_theta : g_theta;
		c_phi = (ucy(1) > 0.0) ? -c_phi : c_phi;
		c_theta = (ucx(0) < 0.0) ? -c_theta : c_theta;

		std::cout << "Current: [" << c_phi << ";" << c_theta << "]" << std::endl;
		std::cout << "Goal: [" << g_phi << ";" << g_theta << "]" << std::endl << std::endl;
		*/


		Eigen::Vector3d goal_w = calc_goal_rates( gr_sp, gr);
		//double goal_wx = 6.0*(g_phi - c_phi);
		//double goal_wy = 6.0*(g_theta - c_theta);
		//double goal_wz = 6.0*(0.0 - 0.0);	//XXX: Hold with zero for now
		double goal_r1d = p_.gain_ang_r1_p*(msg_goal_joints_.position[0] - r1);
		double goal_r2d = p_.gain_ang_r2_p*(msg_goal_joints_.position[1] - r2);

		ua(0,0) = 0.0;
		ua(1,0) = 0.0;
		ua(2,0) = Ab.z();	//TODO: Something else, maybe: Ab(2,0);	//Z acceleration in body frame
		ua(3,0) = p_.gain_rate_roll_p*(goal_w.x() - bwx);
		ua(4,0) = p_.gain_rate_pitch_p*(goal_w.y() - bwy);
		ua(5,0) = p_.gain_rate_yaw_p*(goal_w.z() - bwz);
		ua(6,0) = p_.gain_rate_r1_p*(goal_r1d - r1d);
		ua(7,0) = p_.gain_rate_r2_p*(goal_r2d - r2d);

		//Calculate the
		calc_Dq(D,
				p_.I0x, p_.I0y, p_.I0z,
				p_.I1x, p_.I1y, p_.I1z,
				p_.I2x, p_.I2y, p_.I2z,
				p_.l0, p_.l1, p_.lc1, p_.lc2,
				p_.m0, p_.m1, p_.m2,
				r1, r2);

		calc_Cqqd(C,
				  p_.I1x, p_.I1y,
				  p_.I2x, p_.I2y,
				  bvx, bvy, bvz, bwx, bwy, bwz,
				  p_.l0, p_.l1, p_.lc1, p_.lc2,
				  p_.m1, p_.m2,
				  r1, r1d, r2, r2d);

		//calc_Lqd(L);
		//calc_Nq(N, IJ1z, IJ2z, g, gr(2,2), l0, l1, l2, m0, m1, m2, r1, r2);

		tau = D*ua + (C + L)*qd;// + N;

		//XXX: Hardcoded for hex because lazy
		Eigen::MatrixXd M = Eigen::MatrixXd::Constant(8, 8, 0.0);
		calc_motor_map(M);

		Eigen::MatrixXd u = M*tau;

		//ROS_INFO_STREAM("t:" << std::endl << tau);
		//ROS_INFO_STREAM("M:" << std::endl << M);
		//ROS_INFO_STREAM("u:" << std::endl << u);

		for(int i=0; i<p_.motor_num; i++) {
			msg_rc_out.channels[i] = map_pwm(u(i,0));
		}

		msg_r1_out.data = u(p_.motor_num,0);
		msg_r2_out.data = u(p_.motor_num+1,0);

		msg_twist_out.twist.angular.x = goal_w.x();
		msg_twist_out.twist.angular.y = goal_w.y();
		msg_twist_out.twist.angular.z = goal_w.z();
		msg_accel_out.accel.linear.x = ua(0,0);
		msg_accel_out.accel.linear.y = ua(1,0);
		msg_accel_out.accel.linear.z = ua(2,0);
		msg_accel_out.accel.angular.x = ua(3,0);
		msg_accel_out.accel.angular.y = ua(4,0);
		msg_accel_out.accel.angular.z = ua(5,0);

		msg_joints_out.name = msg_state_joints_.name;
		msg_joints_out.position = msg_goal_joints_.position;
		msg_joints_out.velocity.push_back(goal_r1d);
		msg_joints_out.velocity.push_back(goal_r2d);
		msg_joints_out.effort.push_back(ua(6,0));
		msg_joints_out.effort.push_back(ua(7,0));
	} else {
		//Output nothing until the input info is available
		for(int i=0; i<p_.motor_num; i++) {
			msg_rc_out.channels[i] = p_.pwm_min;
		}

		msg_r1_out.data = 0.0;
		msg_r2_out.data = 0.0;
	}

	pub_rc_.publish(msg_rc_out);
	pub_r1_.publish(msg_r1_out);
	pub_r2_.publish(msg_r2_out);
	pub_twist_.publish(msg_twist_out);
	pub_accel_.publish(msg_accel_out);
	pub_joints_.publish(msg_joints_out);
}

int16_t ControllerID::map_pwm(double val) {
	//Constrain from 0 -> 1
	double c = (val > 1.0) ? 1.0 : (val < 0.0) ? 0.0 : val;

	//Scale c to the pwm values
	return int16_t((p_.pwm_max - p_.pwm_min)*c) + p_.pwm_min;
}

void ControllerID::calc_motor_map(Eigen::MatrixXd &M) {
	//TODO: This all needs to be better defined generically
	double arm_ang = M_PI / 3.0;

	double kT = 1.0 / (p_.motor_num * p_.motor_thrust_max);
	double ktx = 1.0 / (2.0 * p_.la * (2.0 * std::sin(arm_ang / 2.0) + 1.0) * p_.motor_thrust_max);
	double kty = 1.0 / (4.0 * p_.la * std::cos(arm_ang) * p_.motor_thrust_max);
	double km = 1.0 / (p_.motor_num * p_.motor_drag_max);
	km = 0;

	//Generate the copter map
	Eigen::MatrixXd cm = Eigen::MatrixXd::Zero(p_.motor_num, 6);
	cm << 0.0, 0.0,  kT, -ktx,  0.0, -km,
		  0.0, 0.0,  kT,  ktx,  0.0,  km,
		  0.0, 0.0,  kT,  ktx, -kty, -km,
		  0.0, 0.0,  kT, -ktx,  kty,  km,
		  0.0, 0.0,  kT, -ktx, -kty,  km,
		  0.0, 0.0,  kT,  ktx,  kty, -km;

	M << cm, Eigen::MatrixXd::Zero(p_.motor_num, p_.link_num),
		 Eigen::MatrixXd::Zero(p_.link_num, 6), Eigen::MatrixXd::Identity(p_.link_num, p_.link_num);
}

Eigen::Vector3d ControllerID::calc_goal_rates(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R) {
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

	//Method derived from px4 attitude controller:
	//DCM from for state and setpoint

	/*
	Eigen::Matrix3d R = q_current->normalized().toRotationMatrix();
	Eigen::Matrix3d R_sp = q_sp->normalized().toRotationMatrix();
	*/

	//Calculate shortest path to goal rotation without yaw (as it's slower than roll/pitch)
	Eigen::Vector3d R_z = R.col(2);
	Eigen::Vector3d R_sp_z = R_sp.col(2);

	//px4: axis and sin(angle) of desired rotation
	//px4: math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);
	Eigen::Vector3d e_R = R.transpose() * R_z.cross(R_sp_z);

	double e_R_z_sin = e_R.norm();
	double e_R_z_cos = R_z.dot(R_sp_z);

	//px4: calculate weight for yaw control
	double yaw_w = R_sp(2,2)*R_sp(2,2);

	//px4: calculate rotation matrix after roll/pitch only rotation
	Eigen::Matrix3d R_rp;

	e_R_z_sin = 0;	//TODO: XXX: Seems to fix issues somewhere

	if(e_R_z_sin > 0) {
		//px4: get axis-angle representation
		Eigen::Vector3d e_R_z_axis = e_R / e_R_z_sin;
		e_R = e_R_z_axis * std::atan2(e_R_z_sin, e_R_z_cos);

		//px4: cross product matrix for e_R_axis
		Eigen::Matrix3d e_R_cp;
		e_R_cp(0,1) = -e_R_z_axis.z();
		e_R_cp(0,2) = e_R_z_axis.y();
		e_R_cp(1,0) = e_R_z_axis.z();
		e_R_cp(1,2) = -e_R_z_axis.x();
		e_R_cp(2,0) = -e_R_z_axis.y();
		e_R_cp(2,1) = e_R_z_axis.x();

		//px4: rotation matrix for roll/pitch only rotation
		R_rp = R * ( I + (e_R_cp * e_R_z_sin) + ( (e_R_cp * e_R_cp) * (1.0 - e_R_z_cos) ) );
	} else {
		//px4: zero roll/pitch rotation
		R_rp = R;
	}

	//XXX: R_rp = R;

	//px4: R_rp and R_sp has the same Z axis, calculate yaw error
	Eigen::Vector3d R_sp_x = R_sp.col(0);
	Eigen::Vector3d R_rp_x = R_rp.col(0);

	Eigen::Vector3d R_rp_c_sp = R_rp_x.cross(R_sp_x);

	//e_R(2) = atan2f(R_rp_c_sp * R_sp_z, R_rp_x * R_sp_x) * yaw_w;
	e_R(2) = std::atan2(R_rp_c_sp.dot(R_sp_z), R_rp_x.dot(R_sp_x)) * yaw_w;

	if(e_R_z_cos < 0) {
		//px4: for large thrust vector rotations use another rotation method:
		//px4: calculate angle and axis for R -> R_sp rotation directly
		Eigen::Vector3d e_R_d;
		Eigen::Quaterniond q_error(R.transpose() * R_sp);

		if(q_error.w() >= 0) {
			Eigen::Vector3d temp_vec;
			temp_vec.x() = q_error.x();
			temp_vec.y() = q_error.y();
			temp_vec.z() = q_error.z();

			e_R_d = temp_vec * 2.0;
		} else {
			Eigen::Vector3d temp_vec;
			temp_vec.x() = -q_error.x();
			temp_vec.y() = -q_error.y();
			temp_vec.z() = -q_error.z();

			e_R_d = temp_vec * 2.0;
		}

		//px4: use fusion of Z axis based rotation and direct rotation
		double direct_w = e_R_z_cos * e_R_z_cos * yaw_w;

		//px4: e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
		e_R = e_R * (1.0 - direct_w) + e_R_d * direct_w;
	}

	//px4: calculate angular rates setpoint
	Eigen::Vector3d rates_sp;
	rates_sp(0) = p_.gain_ang_roll_p * e_R.x();
	rates_sp(1) = p_.gain_ang_pitch_p * e_R.y();
	rates_sp(2) = p_.gain_ang_yaw_p * e_R.z();

	//px4: feed forward yaw setpoint rate	//TODO:?
	//rates_sp.z += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

	return rates_sp;
}

void ControllerID::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	msg_state_odom_ = *msg_in;
}

void ControllerID::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	msg_state_joints_ = *msg_in;
}

void ControllerID::callback_goal_accel(const geometry_msgs::AccelStamped::ConstPtr& msg_in) {
	msg_goal_accel_ = *msg_in;
}

void ControllerID::callback_goal_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	msg_goal_joints_ = *msg_in;
}
