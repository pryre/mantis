#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <controller_aug/controller_aug.h>
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
#include <mantis_controller_aug/ControlParamsConfig.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>

#define CONST_GRAV 9.80665

ControllerAug::ControllerAug() :
	nh_(),
	nhp_("~"),
	p_(&nh_, &nhp_),
	ref_path_(&nhp_),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_use_mav_state_(false),
	param_use_imu_state_(false),
	param_safety_rate_(20.0),
	param_high_level_rate_(20.0),
	param_low_level_rate_(200.0),
	dyncfg_control_settings_(ros::NodeHandle(nhp_, "control_settings")),
	pos_pid_x_(&nhp_, "control/pos/x"),
	pos_pid_y_(&nhp_, "control/pos/y"),
	pos_pid_z_(&nhp_, "control/pos/z"),
	vel_pid_x_(&nhp_, "control/vel/x"),
	vel_pid_y_(&nhp_, "control/vel/y"),
	vel_pid_z_(&nhp_, "control/vel/z"),
	ang_pid_x_(&nhp_, "control/ang/x"),
	ang_pid_y_(&nhp_, "control/ang/y"),
	ang_pid_z_(&nhp_, "control/ang/z"),
	rate_pid_x_(&nhp_, "control/rate/x"),
	rate_pid_y_(&nhp_, "control/rate/y"),
	rate_pid_z_(&nhp_, "control/rate/z"),
	ready_for_flight_(false) {

	bool success = true;

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("wait_for_path", param_wait_for_path_, param_wait_for_path_);
	nhp_.param("use_imu_state", param_use_imu_state_, param_use_imu_state_);
	nhp_.param("use_mav_state", param_use_mav_state_, param_use_mav_state_);
	nhp_.param("safety_rate", param_safety_rate_, param_safety_rate_);
	nhp_.param("high_level_rate", param_high_level_rate_, param_high_level_rate_);
	nhp_.param("low_level_rate", param_low_level_rate_, param_low_level_rate_);

	dyncfg_control_settings_.setCallback(boost::bind(&ControllerAug::callback_cfg_control_settings, this, _1, _2));

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

	/*
	pos_pid_x_.setGains( p_.gain_position_xy_p, p_.gain_position_xy_i, 0.0, 0.0 );
	pos_pid_y_.setGains( p_.gain_position_xy_p, p_.gain_position_xy_i, 0.0, 0.0 );
	pos_pid_z_.setGains( p_.gain_position_z_p, p_.gain_position_z_i, 0.0, 0.0 );
	//pos_pid_x_.setOutputMinMax( -CONST_GRAV / 2.0, CONST_GRAV / 2.0);
	//pos_pid_y_.setOutputMinMax( -CONST_GRAV / 2.0, CONST_GRAV / 2.0);
	//pos_pid_z_.setOutputMinMax( -3.0 * CONST_GRAV / 4.0, 3.0 * CONST_GRAV / 4.0);
	pos_pid_x_.setOutputMinMax( -1.0, 1.0);
	pos_pid_y_.setOutputMinMax( -1.0, 1.0);
	pos_pid_z_.setOutputMinMax( -2.0, 2.0);
	*/


	mixer_ = Eigen::MatrixXd(6,4);
	mixer_ << 1.0,	-1.0,	 0.0,	 1.0, // Motor 1
			  1.0,	 1.0,	 0.0,	-1.0, // Motor 2
			  1.0,	 0.5,	-1.0,	 1.0, // Motor 3
			  1.0,	-0.5,	 1.0,	-1.0, // Motor 4
			  1.0,	-0.5,	-1.0,	-1.0, // Motor 5
			  1.0,	 0.5,	 1.0,	 1.0; // Motor 6

	if(success) {
		ROS_INFO( "Loaded configuration for %i links", int(joints_.size()) );

		pub_rc_ = nhp_.advertise<mavros_msgs::OverrideRCIn>("output/rc", 10);
		pub_joints_ = nhp_.advertise<sensor_msgs::JointState>("output/joints", 10);

		pub_pose_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/base/pose", 10);
		pub_twist_ = nhp_.advertise<geometry_msgs::TwistStamped>("feedback/base/twist", 10);
		pub_accel_ = nhp_.advertise<geometry_msgs::AccelStamped>("feedback/base/accel", 10);
		pub_wrench_ = nhp_.advertise<geometry_msgs::WrenchStamped>("feedback/wrench_compensation", 10);

		sub_state_odom_ = nhp_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &ControllerAug::callback_state_odom, this );
		sub_state_battery_ = nhp_.subscribe<sensor_msgs::BatteryState>( "state/battery", 10, &ControllerAug::callback_state_battery, this );
		sub_state_joints_ = nhp_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &ControllerAug::callback_state_joints, this );

		if(param_use_imu_state_)
			sub_state_imu_ = nhp_.subscribe<sensor_msgs::Imu>( "state/imu", 10, &ControllerAug::callback_state_imu, this );

		if(param_use_mav_state_)
			sub_state_mav_ = nhp_.subscribe<mavros_msgs::State>( "state/mav", 10, &ControllerAug::callback_state_mav, this );

		//XXX: Initialize takeoff goals
		ref_path_.set_latest( Eigen::Vector3d(p_.takeoff_x, p_.takeoff_y, p_.takeoff_z), Eigen::Quaterniond::Identity() );

		ROS_INFO("Augmented dynamics controller loaded. Waiting for inputs:");
		ROS_INFO("    - state");
		ROS_INFO("    - odom");
		ROS_INFO("    - joints");
		ROS_INFO("    - imu");
		ROS_INFO("    - path");
		ROS_INFO("    - battery");

		timer_ready_check_ = nhp_.createTimer(ros::Duration(1.0/param_safety_rate_), &ControllerAug::callback_ready_check, this );

		//Lock the controller until all the inputs are satisfied
		while( !ready_for_flight_ ) {

			if( ( !param_use_mav_state_ ) || ( msg_mav_state_tr_ != ros::Time(0) ) )
				ROS_INFO_ONCE("Augmented dynamics got input: state");

			if( msg_odom_tr_ != ros::Time(0) )
				ROS_INFO_ONCE("Augmented dynamics got input: odom");

			if( msg_joints_tr_ != ros::Time(0) )
				ROS_INFO_ONCE("Augmented dynamics got input: joints");

			if( ( !param_use_imu_state_ ) || ( msg_imu_tr_ != ros::Time(0) ) )
				ROS_INFO_ONCE("Augmented dynamics got input: imu");

			if( ref_path_.received_valid_path() || !param_wait_for_path_ )
				ROS_INFO_ONCE("Augmented dynamics got input: path");

			if( msg_battery_tr_ != ros::Time(0) )
				ROS_INFO_ONCE("Augmented dynamics got input: battery");

			if( !ros::ok() )
				break;

			ros::spinOnce();

			mavros_msgs::OverrideRCIn msg_rc_out;
			for(int i=0; i<p_.motor_num; i++) {
					msg_rc_out.channels[i] = msg_rc_out.CHAN_NOCHANGE;
			}
			pub_rc_.publish(msg_rc_out);

			ros::Rate(param_safety_rate_).sleep();
		}

		ROS_INFO_ONCE("Augmented dynamics got all inputs!");

		//Start the control loops
		timer_high_level_ = nhp_.createTimer(ros::Duration(1.0/param_high_level_rate_), &ControllerAug::callback_low_level, this );
		timer_low_level_ = nhp_.createTimer(ros::Duration(1.0/param_low_level_rate_), &ControllerAug::callback_high_level, this );

		ROS_INFO("Augmented dynamics controller started!");
	} else {
		ROS_WARN("Augmented Dynamics controller shutting down.");
		ros::shutdown();
	}
}

ControllerAug::~ControllerAug() {
}

void ControllerAug::callback_cfg_control_settings(mantis_controller_aug::ControlParamsConfig &config, uint32_t level) {
	param_wait_for_path_ = config.wait_for_path;
	param_track_end_ = config.track_end;
	param_force_compensation_ = config.force_compensation;
	param_use_wa_est_ = config.use_wa_estimate;
	param_coriolis_compensation_ = config.coriolis_compensation;
	param_accurate_end_tracking_ = config.accurate_end_tracking;
	param_reference_feedback_ = config.reference_feedback;
}

void ControllerAug::callback_ready_check(const ros::TimerEvent& e) {
//If we still have all the inputs satisfied
	if( ( ref_path_.received_valid_path() || !param_wait_for_path_ ) &&
		( msg_odom_tr_ != ros::Time(0) ) &&
		( msg_battery_tr_ != ros::Time(0) ) &&
		( msg_joints_tr_ != ros::Time(0) ) &&
		( ( !param_use_imu_state_ ) || ( msg_imu_tr_ != ros::Time(0) ) ) &&
		( ( !param_use_mav_state_ ) || ( ( msg_mav_state_tr_ != ros::Time(0) ) && state_.status_armed() ) ) ) {

		ready_for_flight_ = true;
	} else {
		if(ready_for_flight_) {
			std::string error_msg = "Input error:\n";

			if( !ref_path_.received_valid_path() && param_wait_for_path_ )
				error_msg += "no valid path\n";

			if( msg_odom_tr_ == ros::Time(0) )
				error_msg += "no odometry\n";

			if( msg_battery_tr_ == ros::Time(0) )
				error_msg += "no battery data\n";

			if( msg_joints_tr_ == ros::Time(0) )
				error_msg += "no joint data\n";

			if( ( param_use_imu_state_ ) && ( msg_imu_tr_ == ros::Time(0) ) )
				error_msg += "no imu data\n";

			if( param_use_mav_state_ ) {
				if( msg_mav_state_tr_ == ros::Time(0) ) {
					error_msg += "no state data\n";
				} else if(!state_.status_armed()) {
					error_msg += "not armed\n";
				}
			}

			ROS_ERROR("%s", error_msg.c_str());
		}

		ready_for_flight_ = false;
		state_.update_status_hl_control(false);
	}
}

void ControllerAug::callback_high_level(const ros::TimerEvent& e) {
	double dt = (e.current_real - e.last_real).toSec();

	//Goal States
	Eigen::Affine3d ge_sp = Eigen::Affine3d::Identity();
	Eigen::Vector3d gev_sp = Eigen::Vector3d::Zero();

	//This needs to be checked every loop to keep the fallback pose correct
	if( !ref_path_.get_ref_state(ge_sp, gev_sp, e.current_real) ) {
		//There was an issue setting the path goal
		//Hold latest position

		//XXX: This isn't needed as it is done internally
		//ge_sp = latest_g_sp_;
		//gev_sp = Eigen::Vector3d::Zero();
	}

	//If we still have all the inputs satisfied
	if( ready_for_flight_ ) {

		if( param_track_end_ ) {
			//Compute transform for all joints in the chain to get base to end effector
			Eigen::Affine3d gbe = Eigen::Affine3d::Identity();
			Eigen::MatrixXd Je = Eigen::MatrixXd::Zero(6, state_.rd().size());

			//This hole thing could be done dynamically for any joint
			//Compute transform for all joints in the chain to get base to end effector
			for(int i=0; i<joints_.size(); i++) {
				gbe = gbe * joints_[i].transform();
			}

			//The linear velocity of the base is dependent on the joint velocities the end effector
			//End effector velocity jacobian
			//XXX: Pretty sure this can be done dynamically in dh_parameters
			calc_Je(Je,
					joints_[1].r(),
					joints_[2].r(),
					joints_[2].q());

			//Calculate the tracking offset for the base
			state_.update_g_sp( calc_goal_base_transform(ge_sp, gbe) );

			//Compensate for velocity terms in manipulator movement
			//This is more accurate, but can make things more unstable
			if(param_accurate_end_tracking_) {
				state_.update_gv_sp( calc_goal_base_velocity(gev_sp, state_.g().linear()*gbe.linear(), Je, state_.rd()) );
			} else {
				state_.update_gv_sp( gev_sp );
			}
		} else {
			//Just track the robot base
			state_.update_g_sp( ge_sp );
			state_.update_gv_sp( gev_sp );
		}

		//Trajectory Tracking Controller
		//pos_pid_x_.setGains( p_.gain_position_xy_p, p_.gain_position_xy_i, 0.0, 0.0 );
		//pos_pid_y_.setGains( p_.gain_position_xy_p, p_.gain_position_xy_i, 0.0, 0.0 );
		//pos_pid_z_.setGains( p_.gain_position_z_p, p_.gain_position_z_i, 0.0, 0.0 );

		double a_p_x = pos_pid_x_.step(dt, state_.g_sp().translation().x(), state_.g().translation().x(), state_.wv().x());
		double a_p_y = pos_pid_y_.step(dt, state_.g_sp().translation().y(), state_.g().translation().y(), state_.wv().y());
		double a_p_z = pos_pid_z_.step(dt, state_.g_sp().translation().z(), state_.g().translation().z(), state_.wv().z());

		double a_v_x = vel_pid_x_.step(dt, state_.gv_sp().x(), state_.wv().x());
		double a_v_y = vel_pid_y_.step(dt, state_.gv_sp().y(), state_.wv().y());
		double a_v_z = vel_pid_z_.step(dt, state_.gv_sp().z(), state_.wv().z());

		Eigen::Vector3d al_sp(a_p_x + a_v_x, a_p_y + a_v_y, a_p_z + a_v_z);

		//Add in gravity compensation
		Eigen::Vector3d a_sp = al_sp + Eigen::Vector3d(0.0, 0.0, CONST_GRAV);

		a_sp(2) = (a_sp(2) < 0.1) ? 0.1 : a_sp(2);	//Can't accelerate downward faster than -g (take a little)
		//XXX:
		//	A bit dirty, but this will stop acceleration going less than
		//	-g and will keep the gr_sp from rotating by more than pi/2
		Eigen::Vector3d axy(a_sp.x(), a_sp.y(), 0.0);
		if(axy.norm() > a_sp.z()) {
			double axy_scale = axy.norm() / a_sp.z();
			axy = axy / axy_scale;
			a_sp.segment(0,2) << axy.segment(0,2);
		}
		//TODO: Constrain vertical max accel(?)

		state_.update_a_sp(a_sp);

		state_.update_status_hl_control(true);
	} else {
		state_.update_status_hl_control(false);

		//Reset the PIDs and keep the integrator down
		pos_pid_x_.reset(state_.g().translation().x());
		pos_pid_y_.reset(state_.g().translation().y());
		pos_pid_z_.reset(state_.g().translation().z());
		vel_pid_x_.reset(state_.wv().x());
		vel_pid_y_.reset(state_.wv().y());
		vel_pid_z_.reset(state_.wv().z());
	}
}


void ControllerAug::callback_low_level(const ros::TimerEvent& e) {
	std::vector<uint16_t> pwm_out(p_.motor_num);	//Allocate space for the number of motors
	std::vector<double> joints_out(p_.manip_num);

	double dt = (e.current_real - e.last_real).toSec();

	if( ready_for_flight_ && state_.status_hl_control() ) {
		//Calculate the corresponding rotation to reach desired acceleration
		//We generate at it base on now yaw, then rotate it to reach the desired orientation
		Eigen::Vector3d an = state_.a_sp().normalized();
		Eigen::Vector3d y_c = state_.g_sp().linear().col(1).normalized();
		Eigen::Vector3d r_sp_x = y_c.cross(an).normalized();
		Eigen::Vector3d r_sp_y = an.cross(r_sp_x).normalized();
		Eigen::Matrix3d gr_sp;
		gr_sp << r_sp_x, r_sp_y, an;

		//Calculate the goal rates to achieve the right acceleration vector
		Eigen::Vector3d e_R = calc_ang_error(gr_sp, state_.g().linear());
		Eigen::Vector3d w_goal;

		//XXX: This is more of a hack to reuse the pid control, but it works out (I think...)
		// Only proportional gains should be used for this controller
		w_goal.x() = ang_pid_x_.step(dt, e_R.x(), 0.0, state_.bw().x());
		w_goal.y() = ang_pid_y_.step(dt, e_R.y(), 0.0, state_.bw().y());
		w_goal.z() = ang_pid_z_.step(dt, e_R.z(), 0.0, state_.bw().z());

		//Calculate the normalized angular accelerations
		Eigen::Vector3d wan;
		wan.x() = rate_pid_x_.step(dt, w_goal.x(), state_.bw().x());
		wan.y() = rate_pid_y_.step(dt, w_goal.y(), state_.bw().y());
		wan.z() = rate_pid_z_.step(dt, w_goal.z(), state_.bw().z());

		//Calculate Abz such that it doesn't apply too much thrust until fully rotated
		Eigen::Vector3d body_z = state_.g().linear()*Eigen::Vector3d::UnitZ();
		double az_scale = state_.a_sp().z() / body_z.z();
		Eigen::Vector3d abz_accel = az_scale*body_z;

		//Calculate the desired amount of thrust to meet the accel vector
		double rpm_max = p_.motor_kv * state_.voltage();	//Get the theoretical maximum rpm at the current battery voltage
		double thrust_single = p_.rpm_thrust_m * rpm_max + p_.rpm_thrust_c;	//Use the RPM to calculate maximum thrust
		double kT = 1.0 / (p_.motor_num * thrust_single);
		double uav_mass = p_.m0 + p_.m1 + p_.m2;

		//Build the normalized thrust and rotational accel input vector
		// F_Treq = mass * accel_req
		// Tn = F_Treq * (1/(6*Tmax))
		double Tn = (uav_mass * abz_accel.norm()) * kT;
		Eigen::VectorXd uan(4);
		uan << Tn, wan;

		//==-- Calculate forces induced by the arm
		Eigen::VectorXd tau(state_.num());

		if(param_force_compensation_) {
			double arm_ang = M_PI / 3.0;
			double ktx = 1.0 / (2.0 * p_.la * (2.0 * std::sin(arm_ang / 2.0) + 1.0) * thrust_single);
			double kty = 1.0 / (4.0 * p_.la * std::cos(arm_ang  / 2.0) * thrust_single);
			double km = -1.0 / (p_.motor_num * p_.motor_drag_max);

			//Need to copy this data to access it directly
			Eigen::VectorXd r = state_.r();
			Eigen::VectorXd rd = state_.rd();
			Eigen::VectorXd rdd = state_.rdd();
			Eigen::VectorXd bw = state_.bw();
			Eigen::VectorXd bv = state_.bv();

			Eigen::VectorXd ua(state_.num());	//vd(6,1) + rdd(2,1)
			ua(0) = 0.0;
			ua(1) = 0.0;
			ua(2) = abz_accel.norm();	//XXX: Maybe should be 0.0 (?)
			//ua(2) = A.norm();
			if(param_use_wa_est_) {
				ua.segment(3,3) << state_.bwa();
			} else {
				//XXX: Not having this enabled can cause instabilities
				ua.segment(3,3) << Eigen::Vector3d::Zero();
			}
			ua.segment(6,p_.manip_num) << rdd;

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

			//XXX: Hardcoded for hex in function because lazy
			//Eigen::MatrixXd M = Eigen::MatrixXd::Zero(p_.motor_num + p_.manip_num, state_.num());
			//calc_motor_map(M);

			//Eigen::MatrixXd u = M*tau;

			//Normalize and add in the augmentation force calcs
			Eigen::VectorXd uaug(4);
			uaug(0) = 0.0; //Thrust augmentation is done above, otherwise: tau(2)*kT; //vertical accel compensation
			uaug(1) = tau(3)*ktx;//roll accel compensation
			uaug(2) = tau(4)*kty;//pitch accel compensation
			uaug(3) = tau(5)*km;//yaw accel compensation

			uan += uaug;
		}

		//Perform the motor mixing
		Eigen::MatrixXd u = mixer_*uan;

		//TODO: Should do some input scaling and constraint checking

		//Calculate goal PWM values to generate desired torque
		for(int i=0; i<p_.motor_num; i++) {
			pwm_out[i] = map_pwm(u(i));
		}

		//for(int i=0; i<p_.manip_num; i++) {
		//	joints_out[i] = u(p_.motor_num + i);
		//}

		if(param_reference_feedback_)
			message_output_feedback(e.current_real, state_.g_sp(), state_.a_sp(), gr_sp, w_goal, tau);
	} else {
		//Output minimums until the input info is available
		for(int i=0; i<p_.motor_num; i++) {
			pwm_out[i] = p_.pwm_min;
		}

		for(int i=0; i<p_.manip_num; i++) {
			joints_out[i] = 0.0;
		}
	}

	message_output_control(e.current_real, pwm_out, joints_out);
}

double ControllerAug::double_clamp(const double v, const double min, const double max) {
	return (v < min) ? min : (v > max) ? max : v;
}

void ControllerAug::matrix_clamp(Eigen::MatrixXd m, const double min, const double max) {
	for(int i=0; i<m.rows(); i++) {
		for(int j=0; j<m.cols(); j++) {
			m(i,j) = double_clamp(m(i,j), min, max);
		}
	}
}

Eigen::VectorXd ControllerAug::vector_interlace(const Eigen::VectorXd a, const Eigen::VectorXd b) {
	ROS_ASSERT_MSG(a.size() == b.size(), "Vectors to be interlaced must be same size (a=%i,b=%i)", int(a.size()),  int(b.size()));

	Eigen::VectorXd c = Eigen::VectorXd::Zero(2*a.size());

	for(int i=0; i<a.size(); i++) {
		c.segment(2*i,2) << a(i), b(i);
	}

	return c;
}

Eigen::Vector3d ControllerAug::position_from_msg(const geometry_msgs::Point p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond ControllerAug::quaternion_from_msg(const geometry_msgs::Quaternion q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Affine3d ControllerAug::affine_from_msg(const geometry_msgs::Pose pose) {
		Eigen::Affine3d a;

		a.translation() << position_from_msg(pose.position);
		a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

		return a;
}

Eigen::Affine3d ControllerAug::calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe) {
	//Use this yaw only rotation to set the direction of the base (and thus end effector)
	Eigen::Matrix3d br_sp = extract_yaw_component(ge_sp.linear());

	//Construct the true end effector
	Eigen::Affine3d sp = Eigen::Affine3d::Identity();
	sp.linear() = br_sp*gbe.linear();
	sp.translation() = ge_sp.translation();

	//Use the inverse transform to get the true base transform
	return sp*gbe.inverse();
}

Eigen::Vector3d ControllerAug::calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::MatrixXd &Je, const Eigen::VectorXd &rd) {
	//Velocity of the end effector in the end effector frame
	Eigen::VectorXd vbe = Je*rd;

	//Eigen::Affine3d Vbe;
	//Vbe.translation() << vbe.segment(0,3);
	//Vbe.linear() << vee_up(vbe.segment(3,3));

	//Get linear velocity in the world frame
	Eigen::Vector3d Ve = Re*vbe.segment(0,3);

	//Only care about the translation movements
	return gev_sp - Ve;
}

Eigen::Matrix3d ControllerAug::extract_yaw_component(const Eigen::Matrix3d r) {
	Eigen::Vector3d sp_x = Eigen::Vector3d::UnitX();
	Eigen::Vector3d sp_y = Eigen::Vector3d::UnitY();

	//As long as y isn't straight up
	if(r.col(1) != Eigen::Vector3d::UnitZ()) {
		//If we have ||roll|| > 90Deg
		Eigen::Vector3d y_c;
		if(r(2,2) > 0.0) {
			y_c = r.col(1);
		} else {
			y_c = -r.col(1);
		}

		sp_x = y_c.cross(Eigen::Vector3d::UnitZ());
		sp_y = Eigen::Vector3d::UnitZ().cross(sp_x);
	} else { //Use X-axis for the edge-case
		Eigen::Vector3d x_c = r.col(0);

		sp_y = Eigen::Vector3d::UnitZ().cross(x_c);
		sp_x = sp_y.cross(Eigen::Vector3d::UnitZ());
	}

	//Get the pure yaw rotation
	Eigen::Matrix3d ry;
	ry << sp_x.normalized(),
		  sp_y.normalized(),
		  Eigen::Vector3d::UnitZ();

	return ry;
}

Eigen::Vector3d ControllerAug::calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R) {
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

	return e_R;
}

int16_t ControllerAug::map_pwm(double val) {
	//Constrain from 0 -> 1
	//double c = (val > 1.0) ? 1.0 : (val < 0.0) ? 0.0 : val;
	double c = double_clamp(val, 0.0, 1.0);

	//Scale c to the pwm values
	return int16_t((p_.pwm_max - p_.pwm_min)*c) + p_.pwm_min;
}
/*
void ControllerAug::calc_motor_map(Eigen::MatrixXd &M) {
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

	M << cm, Eigen::MatrixXd::Zero(p_.motor_num, p_.manip_num),
		 Eigen::MatrixXd::Zero(p_.manip_num, 6), Eigen::MatrixXd::Identity(p_.manip_num, p_.manip_num);
}
*/
void ControllerAug::calc_motor_map(Eigen::MatrixXd &M) {
	//TODO: This all needs to be better defined generically
	double arm_ang = M_PI / 3.0;

	//Calculate the thrust curve
	double rpm_max = p_.motor_kv * state_.voltage();	//Get the theoretical maximum rpm at the current battery voltage
	double thrust_max = p_.rpm_thrust_m * rpm_max + p_.rpm_thrust_c;	//Use the RPM to calculate maximum thrust

	double kT = 1.0 / (p_.motor_num * thrust_max);
	double ktx = 1.0 / (2.0 * p_.la * (2.0 * std::sin(arm_ang / 2.0) + 1.0) * thrust_max);
	double kty = 1.0 / (4.0 * p_.la * std::cos(arm_ang  / 2.0) * thrust_max);
	double km = -1.0 / (p_.motor_num * p_.motor_drag_max);

	//Generate the copter map
	Eigen::MatrixXd cm = Eigen::MatrixXd::Zero(p_.motor_num, 6);
	cm << 0.0, 0.0,  kT, -ktx,  0.0, -km,
		  0.0, 0.0,  kT,  ktx,  0.0,  km,
		  0.0, 0.0,  kT,  ktx, -kty, -km,
		  0.0, 0.0,  kT, -ktx,  kty,  km,
		  0.0, 0.0,  kT, -ktx, -kty,  km,
		  0.0, 0.0,  kT,  ktx,  kty, -km;

	M << cm, Eigen::MatrixXd::Zero(p_.motor_num, p_.manip_num),
		 Eigen::MatrixXd::Zero(p_.manip_num, 6), Eigen::MatrixXd::Identity(p_.manip_num, p_.manip_num);
}

void ControllerAug::message_output_control(const ros::Time t, const std::vector<uint16_t> &pwm, const std::vector<double> &joints) {
	mavros_msgs::OverrideRCIn msg_rc_out;
	sensor_msgs::JointState msg_joints_out;

	//Prepare headers
	//msg_rc_out.header.stamp = t;
	//msg_rc_out.header.frame_id = param_frame_id_;
	msg_joints_out.header.stamp = t;
	msg_joints_out.header.frame_id = param_model_id_;

	//Insert control data
	ROS_ASSERT_MSG(pwm.size() <= 8, "Supported number of motors is 8 (%i)", int(pwm.size()));
	for(int i=0; i<8; i++) {
		if( i<pwm.size() ) {
			msg_rc_out.channels[i] = pwm[i];
		} else {
			msg_rc_out.channels[i] = msg_rc_out.CHAN_NOCHANGE;
		}
	}

	/* TODO:
	msg_joints_out.name = msg_state_joints_.name;
	msg_joints_out.position = msg_state_joints_.position;
	msg_joints_out.effort = joints;
	*/
	//Publish messages
	pub_rc_.publish(msg_rc_out);
	pub_joints_.publish(msg_joints_out);
}

void ControllerAug::message_output_feedback(const ros::Time t,
										   const Eigen::Affine3d &g_sp,
										   const Eigen::Vector3d &pa,
										   const Eigen::Matrix3d &r_sp,
										   const Eigen::Vector3d &g_bw,
										   const Eigen::VectorXd &tau) {
	geometry_msgs::PoseStamped msg_pose_out;
	geometry_msgs::TwistStamped msg_twist_out;
	geometry_msgs::AccelStamped msg_accel_out;
	geometry_msgs::WrenchStamped msg_wrench_out;

	//Prepare headers
	msg_pose_out.header.stamp = t;
	msg_pose_out.header.frame_id = param_frame_id_;
	msg_twist_out.header.stamp = t;
	msg_twist_out.header.frame_id = param_frame_id_;
	msg_accel_out.header.stamp = t;
	msg_accel_out.header.frame_id = param_frame_id_;
	msg_wrench_out.header.stamp = t;
	msg_wrench_out.header.frame_id = param_model_id_;

	//Insert feedback data
	Eigen::Quaterniond r_sp_q(r_sp);
	r_sp_q.normalize();
	msg_pose_out.pose.position.x = g_sp.translation().x();
	msg_pose_out.pose.position.y = g_sp.translation().y();
	msg_pose_out.pose.position.z = g_sp.translation().z();
	msg_pose_out.pose.orientation.w = r_sp_q.w();
	msg_pose_out.pose.orientation.x = r_sp_q.x();
	msg_pose_out.pose.orientation.y = r_sp_q.y();
	msg_pose_out.pose.orientation.z = r_sp_q.z();

	msg_twist_out.twist.angular.x = g_bw.x();
	msg_twist_out.twist.angular.y = g_bw.y();
	msg_twist_out.twist.angular.z = g_bw.z();

	msg_accel_out.accel.linear.x = pa.x();
	msg_accel_out.accel.linear.y = pa.y();
	msg_accel_out.accel.linear.z = pa.z();

	msg_wrench_out.wrench.force.x = tau(0);
	msg_wrench_out.wrench.force.y = tau(1);
	msg_wrench_out.wrench.force.z = tau(2);
	msg_wrench_out.wrench.torque.x = tau(3);
	msg_wrench_out.wrench.torque.y = tau(4);
	msg_wrench_out.wrench.torque.z = tau(5);

	//Publish messages
	pub_pose_.publish(msg_pose_out);
	pub_twist_.publish(msg_twist_out);
	pub_accel_.publish(msg_accel_out);
	pub_wrench_.publish(msg_wrench_out);
}

void ControllerAug::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_imu_tr_).toSec();
	msg_odom_tr_ = msg_in->header.stamp;

	//Update pose
	Eigen::Affine3d g = affine_from_msg(msg_in->pose.pose);
	state_.update_g( g );

	//Update body velocities
	Eigen::Vector3d bv(msg_in->twist.twist.linear.x,
					   msg_in->twist.twist.linear.y,
					   msg_in->twist.twist.linear.z);
	state_.update_bv( bv );

	if( !param_use_imu_state_ ) {
		Eigen::Vector3d bw = Eigen::Vector3d(msg_in->twist.twist.angular.x,
											 msg_in->twist.twist.angular.y,
											 msg_in->twist.twist.angular.z);

		state_.update_bw( bw, dt );
	}
}

void ControllerAug::callback_state_battery(const sensor_msgs::BatteryState::ConstPtr& msg_in) {
	msg_battery_tr_ = msg_in->header.stamp;

	//XXX: Need to do this as the straight voltage reading is too slow
	double voltage = 0.0;
	for(int i=0; i<msg_in->cell_voltage.size(); i++) {
		voltage += msg_in->cell_voltage[i];
	}

	state_.update_voltage( voltage );
}

void ControllerAug::callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_imu_tr_).toSec();
	msg_imu_tr_ = msg_in->header.stamp;

	if( param_use_imu_state_ ) {
		Eigen::Vector3d bw = Eigen::Vector3d(msg_in->angular_velocity.x,
											 msg_in->angular_velocity.y,
											 msg_in->angular_velocity.z);
		state_.update_bw( bw, dt );
	}
}

void ControllerAug::callback_state_mav(const mavros_msgs::State::ConstPtr& msg_in) {
	msg_mav_state_tr_ = msg_in->header.stamp;

	state_.update_status_armed(msg_in->armed);
}

void ControllerAug::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
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
