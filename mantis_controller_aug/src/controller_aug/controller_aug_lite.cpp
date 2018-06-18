#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <controller_aug/controller_aug_lite.h>
#include <controller_aug/controller_aug_params.h>
#include <controller_aug/se_tools.h>
#include <controller_aug/dynamics/calc_Dq.h>
#include <controller_aug/dynamics/calc_Cqqd.h>
#include <controller_aug/dynamics/calc_Lqd.h>
#include <controller_aug/dynamics/calc_Jj2.h>
#include <controller_aug/dynamics/calc_Je.h>
#include <pid_controller_lib/pidController.h>
#include <dh_parameters/dh_parameters.h>
#include <mantis_paths/path_extract.h>
#include <mantis_controller_aug/ControlParamsLiteConfig.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

#define CONST_GRAV 9.80665

ControllerAugLite::ControllerAugLite() :
	nh_(),
	nhp_("~"),
	p_(&nh_, &nhp_),
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

	//Load the robot parameters
	p_.load();	//TODO have this give a success if loaded correctly

	//Load in the link definitions
	for(int i=0; i<p_.body_num; i++) {
		DHParameters dh( &nh_, "body/b" + std::to_string(i) + "/link" );

		if( dh.is_valid() ) {
			joints_.push_back(dh);
		} else {
			ROS_FATAL("Error loading joint %i", int(i));
			success = false;
			break;
		}
	}

	state_.init(p_.manip_num);	//Initialize the state container

	uaug_f_ = Eigen::Vector3d::Zero();

	if(success) {
		ROS_INFO( "Loaded configuration for %i links", int(joints_.size()) );

		pub_np_force_ = nhp_.advertise<mavros_msgs::ActuatorControl>("output/normalized_payload_torque", 10);
		pub_wrench_ = nhp_.advertise<geometry_msgs::WrenchStamped>("feedback/wrench_compensation", 10);

		sub_state_battery_ = nhp_.subscribe<sensor_msgs::BatteryState>( "state/battery", 10, &ControllerAugLite::callback_state_battery, this );
		sub_state_joints_ = nhp_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &ControllerAugLite::callback_state_joints, this );
		sub_state_attitude_target_ = nhp_.subscribe<mavros_msgs::AttitudeTarget>( "state/attitude_target", 10, &ControllerAugLite::callback_state_attitude_target, this );
		sub_state_imu_ = nhp_.subscribe<sensor_msgs::Imu>( "state/imu", 10, &ControllerAugLite::callback_state_imu, this );

		ROS_INFO("Augmented dynamics controller loaded. Waiting for inputs:");
		ROS_INFO("    - attitude target");
		ROS_INFO("    - joints");
		ROS_INFO("    - battery");

		//Lock the controller until all the inputs are satisfied
		while( ros::ok() &&
			   ( ( msg_attitude_target_tr_ == ros::Time(0) ) ||
			     ( msg_joints_tr_ == ros::Time(0) ) ||
				 ( msg_battery_tr_ == ros::Time(0) ) ) ) {

			if( msg_attitude_target_tr_ != ros::Time(0) )
				ROS_INFO_ONCE("Augmented dynamics got input: attitude target");

			if( msg_joints_tr_ != ros::Time(0) )
				ROS_INFO_ONCE("Augmented dynamics got input: joints");

			if( msg_battery_tr_ != ros::Time(0) )
				ROS_INFO_ONCE("Augmented dynamics got input: battery");

			ros::spinOnce();
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

	//==-- Calculate forces induced by the arm
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(state_.num());
	Eigen::Vector3d uaug_out = Eigen::Vector3d::Zero();

	if(param_force_compensation_) {
		//Calculate the desired amount of thrust to meet the accel vector
		double arm_ang = M_PI / 3.0;
		double rpm_max = p_.motor_kv * state_.voltage();	//Get the theoretical maximum rpm at the current battery voltage
		double thrust_single = p_.rpm_thrust_m * rpm_max + p_.rpm_thrust_c;	//Use the RPM to calculate maximum thrust
		double kT = 1.0 / (p_.motor_num * thrust_single);
		double ktx = 1.0 / (2.0 * p_.la * (2.0 * std::sin(arm_ang / 2.0) + 1.0) * thrust_single);
		double kty = 1.0 / (4.0 * p_.la * std::cos(arm_ang  / 2.0) * thrust_single);
		double km = -1.0 / (p_.motor_num * p_.motor_drag_max);
		double uav_mass = p_.m0 + p_.m1 + p_.m2;

		double accel_z = cmd_throttle_ / (kT * uav_mass);

		//Need to copy this data to access it directly
		Eigen::VectorXd r = state_.r();
		Eigen::VectorXd rd = state_.rd();
		//Eigen::VectorXd rdd = state_.rdd();
		Eigen::VectorXd bw = state_.bw();
		Eigen::VectorXd bv = state_.bv();

		//Use this to account for acceleration / translational forces
		//  but assume that all rotations want to maintain 0 acceleration
		Eigen::VectorXd ua = Eigen::VectorXd::Zero(state_.num());	//vd(6,1) + rdd(2,1)
		//ua(0) = 0.0;
		//ua(1) = 0.0;
		ua(2) = accel_z;
		//ua.segment(3,3) << Eigen::Vector3d::Zero();
		//ua.segment(6,p_.manip_num) << rdd;

		//Calculate dynamics matricies
		Eigen::MatrixXd D = Eigen::MatrixXd::Zero(state_.num(), state_.num());
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(state_.num(), state_.num());
		Eigen::MatrixXd L = Eigen::MatrixXd::Zero(state_.num(), state_.num());

		//XXX: Take care!
		double l0 = joints_[0].d();
		double l1 = joints_[1].r();

		calc_Dq(D,
				p_.I0x, p_.I0y, p_.I0z,
				p_.I1x, p_.I1y, p_.I1z,
				p_.I2x, p_.I2y, p_.I2z,
				l0, l1, p_.lc1, p_.lc2,
				p_.m0, p_.m1, p_.m2,
				r(0), r(1));

		calc_Cqqd(C,
				  p_.I1x, p_.I1y,
				  p_.I2x, p_.I2y,
				  bv(0), bv(1), bv(2), bw(0), bw(1), bw(2),
				  l0, l1, p_.lc1, p_.lc2,
				  p_.m1, p_.m2,
				  r(0), rd(0), r(1), rd(1));


		if(param_coriolis_compensation_) {
			tau = D*ua + (C + L)*state_.qd();
		} else {
			tau = D*ua;
		}

		//Normalize and add in the augmentation force calcs
		Eigen::VectorXd uaug(3);
		uaug(0) = tau(3)*ktx;//roll accel compensation
		uaug(1) = tau(4)*kty;//pitch accel compensation
		uaug(2) = tau(5)*km;//yaw accel compensation

		//Low level filter to help reduce issues caused by low level oscilations in the other controllers
		uaug_f_ = param_force_comp_alpha_*uaug + (1.0-param_force_comp_alpha_)*uaug_f_;

		uaug_out = uaug_f_;
	}

	if(param_reference_feedback_) {
		geometry_msgs::WrenchStamped msg_wrench_out;
		msg_wrench_out.header.stamp = e.current_real;
		msg_wrench_out.header.frame_id = param_model_id_;

		msg_wrench_out.wrench.force.x = tau(0);
		msg_wrench_out.wrench.force.y = tau(1);
		msg_wrench_out.wrench.force.z = tau(2);
		msg_wrench_out.wrench.torque.x = tau(3);
		msg_wrench_out.wrench.torque.y = tau(4);
		msg_wrench_out.wrench.torque.z = tau(5);

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

void ControllerAugLite::callback_state_battery(const sensor_msgs::BatteryState::ConstPtr& msg_in) {
	msg_battery_tr_ = msg_in->header.stamp;

	//XXX: Need to do this as the straight voltage reading is too slow
	double voltage = 0.0;
	for(int i=0; i<msg_in->cell_voltage.size(); i++) {
		voltage += msg_in->cell_voltage[i];
	}

	state_.update_voltage( voltage );
}

void ControllerAugLite::callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_imu_tr_).toSec();
	msg_imu_tr_ = msg_in->header.stamp;

	Eigen::Vector3d bw = Eigen::Vector3d(msg_in->angular_velocity.x,
										 msg_in->angular_velocity.y,
										 msg_in->angular_velocity.z);
	state_.update_bw( bw, dt );
}

void ControllerAugLite::callback_state_attitude_target(const mavros_msgs::AttitudeTarget::ConstPtr& msg_in) {
	msg_attitude_target_tr_ = msg_in->header.stamp;

	cmd_throttle_ = msg_in->thrust;
}

void ControllerAugLite::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_joints_tr_).toSec();

	//Only updating if dt is acceptable
	if( (dt > 0.0) && (dt < 1.0) ) {
		for(int i=0; i<joints_.size(); i++) {
			//Only update the dynamic links
			if(joints_[i].jt() != DHParameters::JointType::Static) {
				for(int j=0; j<msg_in->name.size(); j++) {
					//If we have the right joint
					if(joints_[i].name() == msg_in->name[j])
						joints_[i].update(msg_in->position[j], msg_in->velocity[j], dt);
				}
			}
		}
	}

	int jc = 0;
	Eigen::VectorXd r(state_.r().size());
	Eigen::VectorXd rd(state_.rd().size());
	Eigen::VectorXd rdd(state_.rdd().size());

	for(int i=0; i<joints_.size(); i++) {
		//Only update the dynamic links
		if(joints_[i].jt() != DHParameters::JointType::Static) {
			r(jc) = joints_[i].q();
			rd(jc) = joints_[i].qd();
			rdd(jc) = joints_[i].qdd();

			jc++;
		}
	}

	state_.update_r(r);
	state_.update_rd(rd);
	state_.update_rdd(rdd);

	msg_joints_tr_ = msg_in->header.stamp;
}
