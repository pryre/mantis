#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <controller_aug/se_tools.h>
#include <controller_aug/controller_aug_lite.h>
#include <mantis_controller_aug/ControlParamsLiteConfig.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/WrenchStamped.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

#define CONST_GRAV 9.80665

ControllerAugLite::ControllerAugLite() :
	nh_(),
	nhp_("~"),
	p_(&nh_),
	s_(&nh_),
	solver_(&p_, &s_),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_est_rate_(50.0),
	cmd_throttle_(0.0),
	dyncfg_control_settings_(ros::NodeHandle(nhp_, "control_settings")) {

	bool success = true;

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("estimator_rate", param_est_rate_, param_est_rate_);

	dyncfg_control_settings_.setCallback(boost::bind(&ControllerAugLite::callback_cfg_control_settings, this, _1, _2));

	//Wait here for parameters to be loaded
	while( ros::ok() && ( !p_.ok() ) ) {
			ros::spinOnce();
			ros::Rate(param_est_rate_).sleep();
	}

	uaug_f_ = Eigen::Vector3d::Zero();

	if(success) {
		pub_np_force_ = nhp_.advertise<mavros_msgs::ActuatorControl>("output/normalized_payload_torque", 10);
		pub_wrench_ = nhp_.advertise<geometry_msgs::WrenchStamped>("feedback/wrench_compensation", 10);

		sub_state_attitude_target_ = nhp_.subscribe<mavros_msgs::AttitudeTarget>( "command/attitude", 10, &ControllerAugLite::callback_state_attitude_target, this );

		ROS_INFO("Augmented dynamics controller loaded. Waiting for inputs:");
		ROS_INFO("    - attitude target");
		ROS_INFO("    - state");

		//Lock the controller until all the inputs are satisfied
		while( ros::ok() &&
			   ( ( msg_attitude_target_tr_ == ros::Time(0) ) || ( !s_.ok() ) ) ) {

			if( msg_attitude_target_tr_ != ros::Time(0) )
				ROS_INFO_ONCE("Augmented dynamics got input: attitude target");

			if( s_.ok() )
				ROS_INFO_ONCE("Augmented dynamics got input: state");

			ros::spinOnce();
			ros::Rate(param_est_rate_).sleep();
		}

		ROS_INFO_ONCE("Augmented dynamics got all inputs!");

		//Start the control loops
		timer_est_ = nhp_.createTimer(ros::Duration(1.0/param_est_rate_), &ControllerAugLite::callback_est, this );

		ROS_INFO("Augmented dynamics estimator started!");
	} else {
		ROS_WARN("Augmented Dynamics estimator shutting down.");
		ros::shutdown();
	}
}

ControllerAugLite::~ControllerAugLite() {
}

void ControllerAugLite::callback_cfg_control_settings(mantis_controller_aug::ControlParamsLiteConfig &config, uint32_t level) {
	param_force_compensation_ = config.force_compensation;
	param_force_comp_alpha_ = config.force_comp_filter_a;
	param_coriolis_compensation_ = config.coriolis_compensation;
	param_reference_feedback_ = config.reference_feedback;
}

void ControllerAugLite::callback_est(const ros::TimerEvent& e) {
	double dt = (e.current_real - e.last_real).toSec();

	bool solved = false;

	//==-- Calculate forces induced by the arm
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(solver_.num_states());
	Eigen::Vector3d uaug_out = Eigen::Vector3d::Zero();

	if(param_force_compensation_) {
		//Calculate the desired amount of thrust to meet the accel vector
		double arm_ang = M_PI / 3.0;
		double la = p_.base_arm_length();
		double rpm_max = p_.motor_kv() * s_.voltage();	//Get the theoretical maximum rpm at the current battery voltage
		double thrust_single = p_.rpm_thrust_m() * rpm_max + p_.rpm_thrust_c();	//Use the RPM to calculate maximum thrust
		double kT = 1.0 / (p_.motor_num() * thrust_single);
		double ktx = 1.0 / (2.0 * la * (2.0 * std::sin(arm_ang / 2.0) + 1.0) * thrust_single);
		double kty = 1.0 / (4.0 * la * std::cos(arm_ang  / 2.0) * thrust_single);
		double km = -1.0 / (p_.motor_num() * p_.motor_drag_max());

		double uav_mass = 0.0;
		for(int i=0; i<p_.get_body_num(); i++) {
			uav_mass += p_.body_inertial(i).mass;
		}

		double accel_z = cmd_throttle_ / (kT * uav_mass);

		//Use this to account for acceleration / translational forces
		//  but assume that all rotations want to maintain 0 acceleration

		Eigen::VectorXd ua = Eigen::VectorXd::Zero(solver_.num_states());	//vd(6,1) + rdd(2,1)
		//ua(0) = 0.0;
		//ua(1) = 0.0;
		ua(2) = accel_z;
		//ua.segment(3,3) << Eigen::Vector3d::Zero();
		//ua.segment(6,p_.manip_num) << rdd;

		solved = solver_.solve_inverse_dynamics(tau, ua);

		if(solved) {
			//Normalize and add in the augmentation force calcs
			Eigen::VectorXd uaug(3);
			uaug(0) = tau(3)*ktx;//roll accel compensation
			uaug(1) = tau(4)*kty;//pitch accel compensation
			uaug(2) = tau(5)*km;//yaw accel compensation

			//Low level filter to help reduce issues caused by low level oscilations in the other controllers
			uaug_f_ = param_force_comp_alpha_*uaug + (1.0-param_force_comp_alpha_)*uaug_f_;

			uaug_out = uaug_f_;
		}
	}

	if(param_reference_feedback_) {
		geometry_msgs::WrenchStamped msg_wrench_out;
		msg_wrench_out.header.stamp = e.current_real;
		msg_wrench_out.header.frame_id = param_model_id_;

		if(solved) {
			msg_wrench_out.wrench.force.x = tau(0);
			msg_wrench_out.wrench.force.y = tau(1);
			msg_wrench_out.wrench.force.z = tau(2);
			msg_wrench_out.wrench.torque.x = tau(3);
			msg_wrench_out.wrench.torque.y = tau(4);
			msg_wrench_out.wrench.torque.z = tau(5);
		}

		pub_wrench_.publish(msg_wrench_out);
	}

	mavros_msgs::ActuatorControl msg_np_force_out;
	msg_np_force_out.header.stamp = e.current_real;
	msg_np_force_out.header.frame_id = param_model_id_;

	//Set the message to be a force input only message in the body frame
	msg_np_force_out.group_mix = msg_np_force_out.PX4_MIX_PAYLOAD;

	//Put in the force input
	msg_np_force_out.controls[0] = uaug_out(0);
	msg_np_force_out.controls[1] = uaug_out(1);
	msg_np_force_out.controls[2] = uaug_out(2);

	//Publish!
	pub_np_force_.publish(msg_np_force_out);
}

void ControllerAugLite::callback_state_attitude_target(const mavros_msgs::AttitudeTarget::ConstPtr& msg_in) {
	msg_attitude_target_tr_ = msg_in->header.stamp;

	cmd_throttle_ = msg_in->thrust;
}
