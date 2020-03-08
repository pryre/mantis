/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <mantis_controller_id/ControlParamsConfig.h>
#include <mantis_controller_id/controller_id.h>
#include <mantis_description/se_tools.h>
#include <pid_controller_lib/pidController.h>

//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>

#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>

//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

#define CONST_GRAV 9.80665

ControllerID::ControllerID()
	: nh_()
	, nhp_( "~" )
	, p_( nh_ )
	, s_( nh_, p_ )
	, solver_( p_, s_ )
	, param_frame_id_( "map" )
	, param_model_id_( "mantis_uav" )
	, param_low_level_rate_( 200.0 )
	, dyncfg_control_settings_( ros::NodeHandle( nhp_, "control_settings" ) )
	, dyncfg_angle_gains_( ros::NodeHandle( nhp_, "control/ang" ) )
	, rate_pid_x_( ros::NodeHandle( nhp_, "control/rate/x" ) )
	, rate_pid_y_( ros::NodeHandle( nhp_, "control/rate/y" ) )
	, rate_pid_z_( ros::NodeHandle( nhp_, "control/rate/z" ) )
	, param_ang_gain_( 4.5 )
	, param_ang_yaw_w_( 0.6 )
	, a_sp_( Eigen::Vector3d::Zero() )
	, R_sp_( Eigen::Matrix3d::Identity() )
	, yr_sp_( 0 )
	, tc_sp_( 0 )
	, was_flight_ready_( false )
	, abort_flight_( false )
	, sub_accel_( nhp_, "reference/accel", 1 )
	, sub_attitude_target_( nhp_, "reference/attitude", 1 )
	, sub_sp_sync_( sub_accel_, sub_attitude_target_, 10 ) {

	nhp_.param( "frame_id", param_frame_id_, param_frame_id_ );
	nhp_.param( "model_id", param_model_id_, param_model_id_ );
	nhp_.param( "low_level_rate", param_low_level_rate_, param_low_level_rate_ );

	dyncfg_control_settings_.setCallback(
		boost::bind( &ControllerID::callback_cfg_control_settings, this, _1, _2 ) );
	dyncfg_angle_gains_.setCallback(
		boost::bind( &ControllerID::callback_cfg_angle_gains, this, _1, _2 ) );
	sub_sp_sync_.registerCallback(
		boost::bind( &ControllerID::callback_setpoints, this, _1, _2 ) );

	if ( p_.wait_for_params() && s_.wait_for_state() ) {
		pub_actuators_ = nhp_.advertise<mavros_msgs::ActuatorControl>( "output/actuators", 10 );
		pub_att_target_ = nhp_.advertise<mavros_msgs::AttitudeTarget>( "feedback/attitude", 10 );
		// pub_twist_ =
		// nhp_.advertise<geometry_msgs::TwistStamped>("feedback/twist", 10);
		pub_accel_ = nhp_.advertise<geometry_msgs::AccelStamped>( "feedback/accel", 10 );

		ROS_INFO( "[ControllerID] controller loaded, waiting for state and "
				  "references..." );

		// Lock the controller until all the inputs are satisfied
		mavros_msgs::ActuatorControl msg_act_out;
		msg_act_out.header.frame_id = param_model_id_;
		msg_act_out.group_mix = 0;
		for ( int i = 0; i < 8; i++ ) {
			msg_act_out.controls[i] = 0;
		}

		timer_low_level_ = nhp_.createTimer( ros::Duration( 1.0 / param_low_level_rate_ ),
			&ControllerID::callback_low_level, this );
	} else {
		ROS_WARN( "[ControllerID] controller shutting down." );
		ros::shutdown();
	}
}

ControllerID::~ControllerID() {
}

void ControllerID::callback_cfg_control_settings(
	mantis_controller_id::ControlParamsConfig& config, uint32_t level ) {
	param_setpoint_timeout_ = ros::Duration(
		2.0 / config.setpoint_min_rate ); // XXX: x2 to allow for some jitter
	param_reference_feedback_ = config.reference_feedback;
}

void ControllerID::callback_cfg_angle_gains(
	mantis_controller_id::AngleGainsConfig& config, uint32_t level ) {
	param_ang_gain_ = config.p;
	param_ang_yaw_w_ = config.yaw_w;
}

void ControllerID::callback_setpoints(
	const geometry_msgs::AccelStampedConstPtr& accel,
	const mavros_msgs::AttitudeTargetConstPtr& attitude ) {
	if ( ( accel->header.frame_id == attitude->header.frame_id ) && ( accel->header.frame_id == param_frame_id_ ) ) {
		tc_sp_ = accel->header.stamp;

		// XXX: World frame attitude rotation
		R_sp_ = MDTools::quaternion_from_msg( attitude->orientation );

		// XXX: World frame yaw-rate
		if ( !( attitude->type_mask & attitude->IGNORE_YAW_RATE ) ) {
			yr_sp_ = attitude->body_rate.z;
		} else {
			yr_sp_ = 0.0;
		}

		// XXX: World frame acceleration vector
		a_sp_ = MDTools::vector_from_msg( accel->accel.linear );
		a_sp_.z() += CONST_GRAV;
	} else {
		ROS_WARN_THROTTLE( 2.0, "[ControllerID] Inconsistent frame_id's (param: %s; "
								"accel: %s; att: %s)",
			param_frame_id_.c_str(), accel->header.frame_id.c_str(),
			attitude->header.frame_id.c_str() );
	}
}

void ControllerID::callback_low_level( const ros::TimerEvent& e ) {
	// Allocate space for the number of motors
	std::vector<uint16_t> pwm_out( p_.get(MantisParams::PARAM_MOTOR_NUM) );

	double dt = ( e.current_real - e.last_real ).toSec();
	bool ready = s_.ok() && ( tc_sp_ > ros::Time( 0 ) ) && ( ( ros::Time::now() - tc_sp_ ) < param_setpoint_timeout_ );

	bool success = false;
	Eigen::VectorXd u;

	if ( ready && !abort_flight_ ) {
		was_flight_ready_ = true;
		ROS_INFO_ONCE( "[ControllerID] controller started!" );
		// Calculate the corresponding rotation to reach desired acceleration vector
		Eigen::Vector3d e_R = calc_ang_error( R_sp_, s_.g().linear(), param_ang_yaw_w_ );
		Eigen::Vector3d w_goal = param_ang_gain_ * e_R;

		// XXX: This is more of a hack to reuse the pid control, but it works out
		// Only proportional gains should be used for this controller
		// w_goal.x() = ang_pid_x_.step(dt, e_R.x(), 0.0);
		// w_goal.y() = ang_pid_y_.step(dt, e_R.y(), 0.0);
		// w_goal.z() = ang_pid_z_.step(dt, e_R.z(), 0.0);

		// Calculate the normalized angular accelerations
		Eigen::Vector3d wa;
		wa.x() = rate_pid_x_.step( dt, w_goal.x(), s_.bw().x() );
		wa.y() = rate_pid_y_.step( dt, w_goal.y(), s_.bw().y() );
		wa.z() = rate_pid_z_.step( dt, w_goal.z(), s_.bw().z() );

		// Calculate the required manipulator accelerations
		// Eigen::VectorXd e_r = vector_interlace(r_sp - r,
		// Eigen::VectorXd::Zero(p_.manip_num) - rd);
		// Eigen::VectorXd ra = Kr*e_r;

		// Calculate Abz such that it doesn't apply too much thrust until fully
		// rotated
		Eigen::Vector3d body_z = s_.g().linear() * Eigen::Vector3d::UnitZ();
		double az_scale = a_sp_.z() / body_z.z();
		Eigen::Vector3d abz_accel = az_scale * body_z;

		int num_states = solver_.num_states();
		Eigen::VectorXd tau = Eigen::VectorXd::Zero( num_states );
		Eigen::VectorXd ua = Eigen::VectorXd::Zero( num_states );
		ua( 0 ) = 0.0;
		ua( 1 ) = 0.0;
		ua( 2 ) = abz_accel.norm();
		// ua(2) = a_sp_.norm();
		ua.segment( 3, 3 ) << wa;
		ua.segment( 6, p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) )
			<< Eigen::VectorXd::Zero( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );

		double kT = 0.0;
		double ktx = 0.0;
		double kty = 0.0;
		double ktz = 0.0;

		if ( solver_.solve_inverse_dynamics( tau, ua ) && solver_.calculate_thrust_coeffs( kT, ktx, kty, ktz ) ) {
			Eigen::Vector4d cforces;
			cforces << tau( 2 ) * kT, tau( 3 ) * ktx, tau( 4 ) * kty, tau( 5 ) * ktz;
			u = p_.get(MantisParams::PARAM_MIXER) * cforces;

			if ( param_reference_feedback_ ) {
				mavros_msgs::AttitudeTarget msg_att_out;
				geometry_msgs::AccelStamped msg_accel_out;

				msg_att_out.header.stamp = e.current_real;
				msg_att_out.header.frame_id = param_model_id_;
				msg_accel_out.header = msg_att_out.header;

				msg_att_out.type_mask = msg_att_out.IGNORE_THRUST;
				msg_att_out.orientation = MDTools::quaternion_from_eig( Eigen::Quaterniond( R_sp_ ) );
				msg_att_out.body_rate.x = w_goal.x();
				msg_att_out.body_rate.y = w_goal.y();
				msg_att_out.body_rate.z = w_goal.z();
				msg_att_out.thrust = 0.0;

				msg_accel_out.accel.linear.x = ua( 0 );
				msg_accel_out.accel.linear.y = ua( 1 );
				msg_accel_out.accel.linear.z = ua( 2 );
				msg_accel_out.accel.angular.x = ua( 3 );
				msg_accel_out.accel.angular.y = ua( 4 );
				msg_accel_out.accel.angular.z = ua( 5 );

				pub_att_target_.publish( msg_att_out );
				pub_accel_.publish( msg_accel_out );
			}

			success = true;
		}
	} else {
		// We've just had a drop out, switch to failsafe
		if ( was_flight_ready_ ) {
			abort_flight_ = true;
		}

		if ( abort_flight_ )
			ROS_ERROR_THROTTLE(
				2.0, "[ControllerID] Timeout on flight data, entering failsafe!" );
	}

	message_output_control( e.current_real, u );
}

Eigen::Vector3d ControllerID::calc_ang_error( const Eigen::Matrix3d& R_sp,
	const Eigen::Matrix3d& R,
	const double yaw_w ) {
	/*
  XXX: Shorthand method that doesn't allow for separate yaw tuning
  return quaternion_basis_error(Eigen::Quaterniond(R),
  Eigen::Quaterniond(R_sp));
  */
	Eigen::Quaterniond q( R );
	Eigen::Quaterniond q_sp( R_sp );
	Eigen::Vector3d e_z = R.col( 2 );
	Eigen::Vector3d e_z_d = R_sp.col( 2 );

	Eigen::Quaterniond qd_red = Eigen::Quaterniond::FromTwoVectors( e_z, e_z_d );
	qd_red.normalize();

	// Handle co-linear vectoring cases
	if ( ( fabs( qd_red.x() ) >= ( 1.0 - 1e-5 ) ) || ( fabs( qd_red.y() ) >= ( 1.0 - 1e-5 ) ) ) {
		// They are in opposite directions which presents an ambiguous solution
		// The best we can momenterily is to just accept bad weightings from the
		// mixing and
		// do the 'best' movement possible for 1 time step until things can be
		// calculated
		qd_red = q_sp;
	} else {
		// Transform rotation from current to desired thrust vector into a world
		// frame reduced desired attitude reference
		qd_red = qd_red * q;
	}

	qd_red.normalize();

	// mix full and reduced desired attitude
	Eigen::Quaterniond q_mix = qd_red.inverse() * q_sp;
	q_mix = MDTools::quaternion_scale( q_mix, MDTools::sign_no_zero( q_mix.w() ) );

	// catch numerical problems with the domain of acosf and asinf
	// Reduce the influence of the yaw rotation using the yaw_w factor
	q_mix.w() = cos( yaw_w * acos( MDTools::double_clamp( q_mix.w(), -1.0, 1.0 ) ) );
	q_mix.x() = 0.0;
	q_mix.y() = 0.0;
	q_mix.z() = sin( yaw_w * asin( MDTools::double_clamp( q_mix.z(), -1.0, 1.0 ) ) );
	q_mix.normalize(); // Clean up (clamping saves the prior normalizing step, but
	// just to be safe)

	Eigen::Quaterniond qd = qd_red * q_mix;
	qd.normalize();

	Eigen::Vector3d e_R = MDTools::quaternion_basis_error( q, qd );

	return e_R;
}

void ControllerID::message_output_control( const ros::Time t,
	const Eigen::VectorXd& u ) {
	mavros_msgs::ActuatorControl msg_act_out;
	msg_act_out.header.stamp = ros::Time::now();
	msg_act_out.group_mix = 0;

	// Insert control data
	ROS_ASSERT_MSG( u.size() <= 8, "Supported number of motors is 8" );
	for ( int i = 0; i < 8; i++ ) {
		if ( i < u.size() ) {
			msg_act_out.controls[i] = u( i );
		} else {
			msg_act_out.controls[i] = 0;
		}
	}

	// Publish messages
	pub_actuators_.publish( msg_act_out );
}
