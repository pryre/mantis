#include <ros/ros.h>

#include <controller_id/controller_id.h>
#include <controller_id/controller_id_params.h>
#include <dynamics/calc_Dq.h>
#include <dynamics/calc_Cqqd.h>
#include <dynamics/calc_Lqd.h>

#include <mavros_msgs/RCOut.h>
#include <sensor_msgs/JointState.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

ControllerID::ControllerID() :
	nh_("~"),
	p_(&nh_),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_rate_(100),
	latest_g_sp_(Eigen::Affine3d::Identity()),
	path_hint_(0) {

	p_.load();

	pub_rc_ = nh_.advertise<mavros_msgs::RCOut>("output/rc", 10);
	pub_joints_ = nh_.advertise<sensor_msgs::JointState>("output/joints", 10);

	pub_accel_linear_ = nh_.advertise<geometry_msgs::AccelStamped>("feedback/accel/linear", 10);
	pub_accel_body_ = nh_.advertise<geometry_msgs::AccelStamped>("feedback/accel/body", 10);
	pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("feedback/pose", 10);

	sub_state_odom_ = nh_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &ControllerID::callback_state_odom, this );
	sub_state_joints_ = nh_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &ControllerID::callback_state_joints, this );

	sub_goal_path_ = nh_.subscribe<nav_msgs::Path>( "goal/path", 10, &ControllerID::callback_goal_path, this );
	sub_goal_joints_ = nh_.subscribe<sensor_msgs::JointState>( "goal/joints", 10, &ControllerID::callback_goal_joints, this );

	timer_ = nh_.createTimer(ros::Duration(1.0/param_rate_), &ControllerID::callback_control, this );

	//XXX: Initialize takeoff goals
	latest_g_sp_.translation() = Eigen::Vector3d(p_.takeoff_x, p_.takeoff_y, p_.takeoff_z);

	ROS_INFO("Inverse Dynamics controller loaded.");
}

ControllerID::~ControllerID() {
}

void ControllerID::callback_control(const ros::TimerEvent& e) {
	std::vector<uint16_t> pwm_out(p_.motor_num);	//Allocate space for the number of motors
	std::vector<double> joints_out(p_.link_num);

	if( ( msg_state_odom_.header.stamp != ros::Time(0) ) &&
		( msg_state_joints_.header.stamp != ros::Time(0) ) &&
		( msg_goal_joints_.header.stamp != ros::Time(0) ) ) {

		ROS_INFO_ONCE("Inverse dynamics controller running!");

		double num_states = 6 + p_.link_num;	//XXX: 6 comes from XYZ + Wrpy

		//Goal States
		Eigen::Affine3d g_sp = Eigen::Affine3d::Identity();
		Eigen::Vector3d gtv_sp = Eigen::Vector3d::Zero();
		if( !calc_goal_g_sp(g_sp, gtv_sp, e.current_real) ) {
			//There was an issue setting the path goal
			//Hold latest position
			g_sp = latest_g_sp_;
			gtv_sp = Eigen::Vector3d::Zero();
		}

		//TODO: Should definitely have a trajectory instead
		Eigen::VectorXd r_sp = Eigen::VectorXd::Zero(p_.link_num);

		//Current States
		Eigen::Affine3d g = affine_from_msg(msg_state_odom_.pose.pose);

		Eigen::Vector3d bv(msg_state_odom_.twist.twist.linear.x,
						   msg_state_odom_.twist.twist.linear.y,
						   msg_state_odom_.twist.twist.linear.z);
		Eigen::Vector3d bw(msg_state_odom_.twist.twist.angular.x,
						   msg_state_odom_.twist.twist.angular.y,
						   msg_state_odom_.twist.twist.angular.z);

		Eigen::VectorXd r = Eigen::VectorXd::Zero(p_.link_num);
		Eigen::VectorXd rd = Eigen::VectorXd::Zero(p_.link_num);
		for(int i=0; i<p_.link_num; i++) {
			r_sp(i) = msg_goal_joints_.position[i];

			r(i) = msg_state_joints_.position[i];
			rd(i) = msg_state_joints_.velocity[i];
		}

		Eigen::VectorXd qd = Eigen::VectorXd::Zero(num_states);
		qd << bv, bw, rd;	//Full body-frame velocity state vector

		//Build gain matricies
		Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(3,6);
		for(int i=0; i<(p_.gain_position.size()/2); i++) {
			Kp.block(i,2*i,1,2) << p_.gain_position[2*i], p_.gain_position[(2*i)+1];
		}

		Eigen::MatrixXd Kw = Eigen::MatrixXd::Zero(3,6);
		for(int i=0; i<(p_.gain_rotation.size()/2); i++) {
			Kw.block(i,2*i,1,2) << p_.gain_rotation[2*i], p_.gain_rotation[(2*i)+1];
		}

		Eigen::MatrixXd Kr = Eigen::MatrixXd::Zero(p_.link_num, 2*p_.link_num);
		for(int i=0; i<(p_.gain_manipulator.size()/2); i++) {
			Kr.block(i,2*i,1,2) << p_.gain_manipulator[2*i], p_.gain_manipulator[(2*i)+1];
		}

		//Calculate translation acceleration vector
		Eigen::Vector3d e_p_p = g_sp.translation() - g.translation();
		Eigen::Vector3d e_p_v = gtv_sp - (g.linear().transpose()*bv);
		//matrix_clamp(e_p_p, -p_.vel_max, p_.vel_max);
		Eigen::VectorXd e_p = vector_interlace(e_p_p, e_p_v);
		Eigen::Vector3d Al = Kp*e_p;

		//Add in gravity compensation
		Eigen::Vector3d A = Al + Eigen::Vector3d(0.0, 0.0, 9.80665);

		A(2) = (A(2) < 0.1) ? 0.1 : A(2);	//Can't accelerate downward faster than -g (take a little)
		//XXX:
		//	A bit dirty, but this will stop acceleration going less than
		//	-g and will keep the gr_sp from rotating by more than pi/2
		Eigen::Vector3d Axy(A.x(), A.y(), 0.0);
		if(Axy.norm() > A.z()) {
			double axy_scale = Axy.norm() / A.z();
			Axy = Axy / axy_scale;
			A.segment(0,2) << Axy.segment(0,2);
		}
		//TODO: Constrain vertical max accel(?)


		//Calculate the corresponding rotation to reach desired acceleration
		//Start with the goal yaw as the current yaw
		Eigen::Vector3d yaw_c = g_sp.linear()*Eigen::Vector3d::UnitY();
		yaw_c.z() = 0;
		yaw_c.normalize();	//Flatten the vector to get yaw only

		//Build the setpoint rotation matrix
		Eigen::Matrix3d gr_sp;
		Eigen::Vector3d sp_x = yaw_c.cross(A).normalized();
		Eigen::Vector3d sp_y = A.cross(sp_x).normalized();
		Eigen::Vector3d sp_z = A.normalized();
		gr_sp.col(0) = sp_x;
		gr_sp.col(1) = sp_y;
		gr_sp.col(2) = sp_z;
		gr_sp.normalize();

		//Calculate the goal rates to achieve the right acceleration vector
		Eigen::VectorXd e_w = vector_interlace(calc_ang_error(gr_sp, g.linear()), Eigen::Vector3d::Zero() - bw);
		Eigen::Vector3d wa = Kw*e_w;

		//Calculate the required manipulator accelerations
		Eigen::VectorXd e_r = vector_interlace(r_sp - r, Eigen::VectorXd::Zero(p_.link_num) - rd);
		Eigen::VectorXd ra = Kr*e_r;

		//Calculate Abz such that it doesn't apply too much thrust until fully rotated
		Eigen::Vector3d body_z = g.linear()*Eigen::Vector3d::UnitZ();
		double Az_scale = A.z() / body_z.z();
		Eigen::Vector3d Abz_accel = Az_scale*body_z;

		Eigen::VectorXd ua = Eigen::VectorXd::Zero(num_states);	//vd(6,1) + rdd(2,1)
		ua(0) = 0.0;
		ua(1) = 0.0;
		ua(2) = Abz_accel.norm();
		ua.segment(3,3) << wa;
		ua.segment(6,p_.link_num) << ra;

		//Calculate dynamics matricies
		Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_states, num_states);
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_states, num_states);
		Eigen::MatrixXd L = Eigen::MatrixXd::Zero(num_states, num_states);

		calc_Dq(D,
				p_.I0x, p_.I0y, p_.I0z,
				p_.I1x, p_.I1y, p_.I1z,
				p_.I2x, p_.I2y, p_.I2z,
				p_.l0, p_.l1, p_.lc1, p_.lc2,
				p_.m0, p_.m1, p_.m2,
				r(0), r(1));

		calc_Cqqd(C,
				  p_.I1x, p_.I1y,
				  p_.I2x, p_.I2y,
				  bv(0), bv(1), bv(2), bw(0), bw(1), bw(2),
				  p_.l0, p_.l1, p_.lc1, p_.lc2,
				  p_.m1, p_.m2,
				  r(0), rd(0), r(1), rd(1));

		Eigen::VectorXd tau = D*ua + (C + L)*qd;

		//XXX: Hardcoded for hex in function because lazy
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(p_.motor_num + p_.link_num, num_states);
		calc_motor_map(M);

		Eigen::MatrixXd u = M*tau;

		//Calculate goal PWM values to generate desired torque
		for(int i=0; i<p_.motor_num; i++) {
			pwm_out[i] = map_pwm(u(i));
		}

		for(int i=0; i<p_.link_num; i++) {
			joints_out[i] = u(p_.motor_num + i);
		}

		message_output_feedback(e.current_real, g_sp, Al, ua);
	} else {
		//Output minimums until the input info is available
		for(int i=0; i<p_.motor_num; i++) {
			pwm_out[i] = p_.pwm_min;
		}

		for(int i=0; i<p_.link_num; i++) {
			joints_out[i] = 0.0;
		}
	}

	message_output_control(e.current_real, pwm_out, joints_out);
}

void ControllerID::matrix_clamp(Eigen::MatrixXd m, const double min, const double max) {
	for(int i=0; i<m.rows(); i++) {
		for(int j=0; j<m.cols(); j++) {
			m(i,j) = (m(i,j) < min) ? min : (m(i,j) > max) ? max : m(i,j);
		}
	}
}

Eigen::VectorXd ControllerID::vector_interlace(const Eigen::VectorXd a, const Eigen::VectorXd b) {
	ROS_ASSERT_MSG(a.size() == b.size(), "Vectors to be interlaced must be same size (a=%li,b=%li", a.size(), b.size());

	Eigen::VectorXd c = Eigen::VectorXd::Zero(2*a.size());

	for(int i=0; i<a.size(); i++) {
		c.segment(2*i,2) << a(i), b(i);
	}

	return c;
}

Eigen::Vector3d ControllerID::position_from_msg(const geometry_msgs::Point p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond ControllerID::quaternion_from_msg(const geometry_msgs::Quaternion q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Affine3d ControllerID::affine_from_msg(const geometry_msgs::Pose pose) {
		Eigen::Affine3d a;

		a.translation() << position_from_msg(pose.position);
		a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

		return a;
}

Eigen::Vector3d ControllerID::vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha) {
  return ((1.0 - alpha) * a) + (alpha * b);
}

bool ControllerID::calc_goal_g_sp(Eigen::Affine3d &g_sp, Eigen::Vector3d &v_sp, const ros::Time tc) {
	bool success = false;

	//If we have recieved a path message
	if(msg_goal_path_.header.stamp > ros::Time(0)) {
		ros::Duration duration_start = msg_goal_path_.poses.front().header.stamp - ros::Time(0);
		ros::Duration duration_end = msg_goal_path_.poses.back().header.stamp - ros::Time(0);

		ros::Time ts = msg_goal_path_.header.stamp + duration_start;
		ros::Time tf = msg_goal_path_.header.stamp + duration_end;
		ros::Duration td = duration_end - duration_start;

		//If the current time is within the path time, follow the path
		if((tc >= ts) && (tc < tf)) {
			//Find the lastest point in the path
			int p = path_hint_ + 1;
			bool found = false;
			while( (!found) && ( p < msg_goal_path_.poses.size() ) ) {
				ros::Duration d_l = msg_goal_path_.poses[p - 1].header.stamp - ros::Time(0);
				ros::Duration d_n = msg_goal_path_.poses[p].header.stamp - ros::Time(0);

				if( (tc >= msg_goal_path_.header.stamp + d_l ) &&
					(tc < msg_goal_path_.header.stamp + d_n ) ) {
					//We have the right index + 1
					//so subtract and break;
					p--;
					break;
				}

				p++;
			}

			//Record the index we ues so we can start the time checks there next loop
			path_hint_ = p;

			//Get the last and next points
			Eigen::Affine3d g_l = affine_from_msg(msg_goal_path_.poses[p].pose);
			Eigen::Affine3d g_n = affine_from_msg(msg_goal_path_.poses[p + 1].pose);
			ros::Duration t_l = msg_goal_path_.poses[p].header.stamp - ros::Time(0);
			ros::Duration t_n = msg_goal_path_.poses[p + 1].header.stamp - ros::Time(0);
			ros::Time t_lt = msg_goal_path_.header.stamp + t_l;
			double dts = (t_n - t_l).toSec();	//Time to complete this segment
			double da = (tc - t_lt).toSec();	//Time to alpha
			double alpha = da / dts;

			//Position goal
			g_sp.translation() << vector_lerp(g_l.translation(), g_n.translation(), alpha);
			Eigen::Quaterniond ql_sp(g_l.linear());
			Eigen::Quaterniond qc_sp = ql_sp.slerp(alpha, Eigen::Quaterniond(g_n.linear()));
			g_sp.linear() << qc_sp.toRotationMatrix();

			//Velocity goal
			v_sp = (g_n.translation() -  g_l.translation()) / dts;

			//Update latest setpoint in case we need to hold lastest position
			latest_g_sp_ = g_sp;

			success = true;
		} else {
			path_hint_ = 0;
		}
	}

	return success;
}

Eigen::Vector3d ControllerID::calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R) {
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

	//Method derived from px4 attitude controller:
	//DCM from for state and setpoint

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

	return e_R;
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
	double kty = 1.0 / (4.0 * p_.la * std::cos(arm_ang  / 2.0) * p_.motor_thrust_max);
	double km = -1.0 / (p_.motor_num * p_.motor_drag_max);
	//km = 0;	//TODO: Need to pick motor_drag_max!!!

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

void ControllerID::message_output_control(const ros::Time t, const std::vector<uint16_t> &pwm, const std::vector<double> &joints) {
	mavros_msgs::RCOut msg_rc_out;
	sensor_msgs::JointState msg_joints_out;

	//Prepare headers
	msg_rc_out.header.stamp = t;
	msg_rc_out.header.frame_id = param_frame_id_;
	msg_joints_out.header.stamp = t;
	msg_joints_out.header.frame_id = param_model_id_;

	//Insert control data
	msg_rc_out.channels = pwm;
	msg_joints_out.name = msg_goal_joints_.name;
	msg_joints_out.position = msg_goal_joints_.position;
	msg_joints_out.effort = joints;

	//Publish messages
	pub_rc_.publish(msg_rc_out);
	pub_joints_.publish(msg_joints_out);
}

void ControllerID::message_output_feedback(const ros::Time t, const Eigen::Affine3d &g_sp, const Eigen::Vector3d &pa, const Eigen::VectorXd &ua) {
	geometry_msgs::PoseStamped msg_pose_out;
	geometry_msgs::AccelStamped msg_accel_linear_out;
	geometry_msgs::AccelStamped msg_accel_body_out;

	//Prepare headers
	msg_pose_out.header.stamp = t;
	msg_pose_out.header.frame_id = param_frame_id_;
	msg_accel_linear_out.header.stamp = t;
	msg_accel_linear_out.header.frame_id = param_frame_id_;
	msg_accel_body_out.header.stamp = t;
	msg_accel_body_out.header.frame_id = param_model_id_;

	//Insert feedback data
	Eigen::Quaterniond g_sp_q(g_sp.linear());
	msg_pose_out.pose.position.x = g_sp.translation().x();
	msg_pose_out.pose.position.y = g_sp.translation().y();
	msg_pose_out.pose.position.z = g_sp.translation().z();
	msg_pose_out.pose.orientation.w = g_sp_q.w();
	msg_pose_out.pose.orientation.x = g_sp_q.x();
	msg_pose_out.pose.orientation.y = g_sp_q.y();
	msg_pose_out.pose.orientation.z = g_sp_q.z();

	msg_accel_linear_out.accel.linear.x = pa.x();
	msg_accel_linear_out.accel.linear.y = pa.y();
	msg_accel_linear_out.accel.linear.z = pa.z();

	msg_accel_body_out.accel.linear.x = ua(0);
	msg_accel_body_out.accel.linear.y = ua(1);
	msg_accel_body_out.accel.linear.z = ua(2);
	msg_accel_body_out.accel.angular.x = ua(3);
	msg_accel_body_out.accel.angular.y = ua(4);
	msg_accel_body_out.accel.angular.z = ua(5);

	//Publish messages
	pub_pose_.publish(msg_pose_out);
	pub_accel_linear_.publish(msg_accel_linear_out);
	pub_accel_body_.publish(msg_accel_body_out);
}

void ControllerID::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	msg_state_odom_ = *msg_in;
}

void ControllerID::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	msg_state_joints_ = *msg_in;
}

void ControllerID::callback_goal_path(const nav_msgs::Path::ConstPtr& msg_in) {
	//If there is at least 2 poses in the path
	if(msg_in->poses.size() > 1) {
		//If at least the very last timestamp is in the future, accept path
		if( ( msg_in->header.stamp + ( msg_in->poses.back().header.stamp - ros::Time(0) ) ) > ros::Time::now() ) {
			ROS_INFO("Recieved new path!");
			msg_goal_path_ = *msg_in;
		} else {
			ROS_WARN("Rejecting path, timestamps are too old.");
		}
	} else {
		ROS_WARN("Rejecting path, must be at least 2 poses.");
	}
}

void ControllerID::callback_goal_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	msg_goal_joints_ = *msg_in;
}
