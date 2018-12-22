#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_controller_id/controller_id.h>
#include <pid_controller_lib/pidController.h>
#include <mantis_description/se_tools.h>
#include <mantis_controller_id/ControlParamsConfig.h>

//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>

#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>

//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

#define CONST_GRAV 9.80665

ControllerID::ControllerID() :
	nh_(),
	nhp_("~"),
	p_(nh_),
	s_(nh_, p_),
	solver_(p_, s_),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_low_level_rate_(200.0),
	dyncfg_control_settings_(ros::NodeHandle(nhp_, "control_settings")),
	ang_pid_x_(ros::NodeHandle(nhp_, "control/ang/x")),
	ang_pid_y_(ros::NodeHandle(nhp_, "control/ang/y")),
	ang_pid_z_(ros::NodeHandle(nhp_, "control/ang/z")),
	rate_pid_x_(ros::NodeHandle(nhp_, "control/rate/x")),
	rate_pid_y_(ros::NodeHandle(nhp_, "control/rate/y")),
	rate_pid_z_(ros::NodeHandle(nhp_, "control/rate/z")),
	was_flight_ready_(false),
	abort_flight_(false),
	sub_accel_(nhp_, "reference/accel", 1),
	sub_attitude_target_(nhp_, "reference/attitude", 1),
	sub_sp_sync_(sub_accel_, sub_attitude_target_, 10) {

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("low_level_rate", param_low_level_rate_, param_low_level_rate_);

	dyncfg_control_settings_.setCallback(boost::bind(&ControllerID::callback_cfg_control_settings, this, _1, _2));
	sub_sp_sync_.registerCallback(boost::bind(&ControllerID::callback_setpoints, this, _1, _2));

	a_sp_ = Eigen::Vector3d::Zero();

	if(p_.wait_for_params() && s_.wait_for_state()) {
		pub_actuators_ = nhp_.advertise<mavros_msgs::ActuatorControl>("output/actuators", 10);
		pub_twist_ = nhp_.advertise<geometry_msgs::TwistStamped>("feedback/twist", 10);
		pub_accel_ = nhp_.advertise<geometry_msgs::AccelStamped>("feedback/accel", 10);

		ROS_INFO("[ControllerID] controller loaded, waiting for state and references...");

		//Lock the controller until all the inputs are satisfied
		mavros_msgs::ActuatorControl msg_act_out;
		msg_act_out.header.frame_id = param_model_id_;
		msg_act_out.group_mix = 0;
		for(int i=0; i<8; i++) {
				msg_act_out.controls[i] = 0;
		}

		timer_low_level_ = nhp_.createTimer(ros::Duration(1.0/param_low_level_rate_), &ControllerID::callback_low_level, this );
	} else {
		ROS_WARN("[ControllerID] controller shutting down.");
		ros::shutdown();
	}
}

ControllerID::~ControllerID() {
}

void ControllerID::callback_cfg_control_settings(mantis_controller_id::ControlParamsConfig &config, uint32_t level) {
	param_setpoint_timeout_ = ros::Duration(2.0/config.setpoint_min_rate);	//XXX: x2 to allow for some jitter
	param_reference_feedback_ = config.reference_feedback;
}

void ControllerID::callback_setpoints(const geometry_msgs::AccelStampedConstPtr& accel, const mavros_msgs::AttitudeTargetConstPtr& attitude) {
		if( (accel->header.frame_id == attitude->header.frame_id) && (accel->header.frame_id == param_frame_id_) ) {
			tc_sp_ = accel->header.stamp;

			//XXX: World frame attitude rotation
			R_sp_ = MDTools::quaternion_from_msg(attitude->orientation);

			//XXX: World frame acceleration vector
			a_sp_ = MDTools::vector_from_msg(accel->accel.linear);
			a_sp_.z() += CONST_GRAV;
		} else {
			ROS_WARN_THROTTLE(2.0, "[ControllerID] Inconsistent frame_id's (param: %s; accel: %s; att: %s)", param_frame_id_.c_str(), accel->header.frame_id.c_str(), attitude->header.frame_id.c_str() );
		}
}

void ControllerID::callback_low_level(const ros::TimerEvent& e) {
	std::vector<uint16_t> pwm_out(p_.motor_num());	//Allocate space for the number of motors

	double dt = (e.current_real - e.last_real).toSec();
	bool ready = s_.ok() && ( tc_sp_ > ros::Time(0) ) && ( (ros::Time::now() - tc_sp_) < param_setpoint_timeout_ );

	bool success = false;
	Eigen::VectorXd u;

	if( ready && !abort_flight_ ) {
		was_flight_ready_ = true;
		ROS_INFO_ONCE("[ControllerID] controller started!");
		//Calculate the corresponding rotation to reach desired acceleration vector
		Eigen::Vector3d e_R = calc_ang_error(R_sp_, s_.g().linear());
		Eigen::Vector3d w_goal;

		//XXX: This is more of a hack to reuse the pid control, but it works out
		// Only proportional gains should be used for this controller
		w_goal.x() = ang_pid_x_.step(dt, e_R.x(), 0.0);
		w_goal.y() = ang_pid_y_.step(dt, e_R.y(), 0.0);
		w_goal.z() = ang_pid_z_.step(dt, e_R.z(), 0.0);

		//Calculate the normalized angular accelerations
		Eigen::Vector3d wa;
		wa.x() = rate_pid_x_.step(dt, w_goal.x(), s_.bw().x());
		wa.y() = rate_pid_y_.step(dt, w_goal.y(), s_.bw().y());
		wa.z() = rate_pid_z_.step(dt, w_goal.z(), s_.bw().z());

		//Calculate the required manipulator accelerations
		//Eigen::VectorXd e_r = vector_interlace(r_sp - r, Eigen::VectorXd::Zero(p_.manip_num) - rd);
		//Eigen::VectorXd ra = Kr*e_r;

		//Calculate Abz such that it doesn't apply too much thrust until fully rotated
		Eigen::Vector3d body_z = s_.g().linear()*Eigen::Vector3d::UnitZ();
		double az_scale = a_sp_.z() / body_z.z();
		Eigen::Vector3d abz_accel = az_scale*body_z;

		int num_states = solver_.num_states();
		Eigen::VectorXd tau = Eigen::VectorXd::Zero(num_states);
		Eigen::VectorXd ua = Eigen::VectorXd::Zero(num_states);
		ua(0) = 0.0;
		ua(1) = 0.0;
		ua(2) = abz_accel.norm();
		//ua(2) = a_sp_.norm();
		ua.segment(3,3) << wa;
		ua.segment(6,p_.get_dynamic_joint_num()) << Eigen::VectorXd::Zero( p_.get_dynamic_joint_num() );

		double kT = 0.0;
		double ktx = 0.0;
		double kty = 0.0;
		double ktz = 0.0;

		if( solver_.solve_inverse_dynamics(tau, ua) && solver_.calculate_thrust_coeffs(kT, ktx, kty, ktz)) {
			Eigen::Vector4d cforces;
			cforces << tau(2)*kT, tau(3)*ktx, tau(4)*kty, tau(5)*ktz;
			u = p_.get_mixer()*cforces;

			if(param_reference_feedback_) {
				geometry_msgs::TwistStamped msg_twist_out;
				geometry_msgs::AccelStamped msg_accel_out;

				msg_twist_out.header.stamp = e.current_real;
				msg_twist_out.header.frame_id = param_model_id_;
				msg_accel_out.header = msg_twist_out.header;

				msg_twist_out.twist.linear.x = 0.0;
				msg_twist_out.twist.linear.y = 0.0;
				msg_twist_out.twist.linear.z = 0.0;
				msg_twist_out.twist.angular.x = w_goal.x();
				msg_twist_out.twist.angular.y = w_goal.y();
				msg_twist_out.twist.angular.z = w_goal.z();

				msg_accel_out.accel.linear.x = ua(0);
				msg_accel_out.accel.linear.y = ua(1);
				msg_accel_out.accel.linear.z = ua(2);
				msg_accel_out.accel.angular.x = ua(3);
				msg_accel_out.accel.angular.y = ua(4);
				msg_accel_out.accel.angular.z = ua(5);

				pub_twist_.publish(msg_twist_out);
				pub_accel_.publish(msg_accel_out);
			}

			success = true;
		}
	} else {
		//We've just had a drop out, switch to failsafe
		if(was_flight_ready_) {
			abort_flight_ = true;
		}

		if(abort_flight_)
			ROS_ERROR_THROTTLE( 2.0, "[ControllerID] Timeout on flight data, entering failsafe!" );
	}

	message_output_control(e.current_real, u);
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

	//px4: calculate rotation matrix after roll/pitch only rotation
	Eigen::Matrix3d R_rp;

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

	//px4: R_rp and R_sp has the same Z axis, calculate yaw error
	Eigen::Vector3d R_sp_x = R_sp.col(0);
	Eigen::Vector3d R_rp_x = R_rp.col(0);

	//px4: calculate weight for yaw control
	double yaw_w = e_R_z_cos * e_R_z_cos;

	//ROS_INFO_STREAM("e_R_z_cos: " << e_R_z_cos << std::endl);

	//px4: e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;
	//Eigen::Vector3d R_rp_c_sp = R_rp_x.cross(R_sp_x);
	//e_R(2) = std::atan2(R_rp_c_sp.dot(R_sp_z), R_rp_x.dot(R_sp_x)) * yaw_w;
	e_R(2) = std::atan2( (R_rp_x.cross(R_sp_x)).dot(R_sp_z), R_rp_x.dot(R_sp_x)) * yaw_w;

	if(e_R_z_cos < 0) {
		//px4: for large thrust vector rotations use another rotation method:
		//For normal operation, this implies a roll/pitch change greater than pi
		//Should never be an issue for us
		ROS_WARN_THROTTLE(1.0, "Large thrust vector detected!");
	}

	double rp_max = 5.0;
	double y_max = 0.5;

	e_R(0) = MDTools::double_clamp(e_R(0), -rp_max, rp_max);
	e_R(1) = MDTools::double_clamp(e_R(1), -rp_max, rp_max);
	e_R(2) = MDTools::double_clamp(e_R(2), -y_max, y_max);

	return e_R;
}

void ControllerID::message_output_control(const ros::Time t, const Eigen::VectorXd &u) {
	mavros_msgs::ActuatorControl msg_act_out;
	msg_act_out.header.stamp = ros::Time::now();
	msg_act_out.group_mix = 0;

	//Insert control data
	ROS_ASSERT_MSG(u.size() <= 8, "Supported number of motors is 8");
	for(int i=0; i<8; i++) {
		if( i<u.size() ) {
			msg_act_out.controls[i] = u(i);
		} else {
			msg_act_out.controls[i] = 0;
		}
	}

	//Publish messages
	pub_actuators_.publish(msg_act_out);
}
