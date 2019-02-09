#include <eigen3/Eigen/Dense>

#include <dh_parameters/JointDescription.h>
#include <mantis_description/se_tools.h>
#include <mantis_msgs/State.h>
#include <mantis_params/param_client.h>
#include <mantis_state/state_server.h>
#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

/*
#include "mantis_state/SystemModel.hpp"
#include "mantis_state/OrientationMeasurementModel.hpp"
#include "mantis_state/PositionMeasurementModel.hpp"

#include "ExtendedKalmanFilter.hpp"
#include "UnscentedKalmanFilter.hpp"
*/


namespace MantisState {
Server::Server( void )
	: nh_()
	, nhp_( "~" )
	, p_( nh_ )
	, param_rate_( 0.0 ) {

	//TODO: Setup dynamic reconfigure for all selection parameters and stream timeouts

	nhp_.param( "update_rate", param_rate_, param_rate_ );

	if ( p_.wait_for_params() ) {
		if ( param_rate_ > 0.0 ) {
			// Current States
			g_ = Eigen::Affine3d::Identity();
			bv_ = Eigen::Vector3d::Zero();
			bw_ = Eigen::Vector3d::Zero();
			ba_ = Eigen::Vector3d::Zero();
			//bwa_ = Eigen::Vector3d::Zero();

			// Additional state information
			voltage_ = 0.0;
			mav_ready_ = false;

			// Parameter-reliant states
			r_ = Eigen::VectorXd::Zero( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
			rd_ = Eigen::VectorXd::Zero( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
			//rdd_ = Eigen::VectorXd::Zero( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );

			sub_state_odom_ = nh_.subscribe<nav_msgs::Odometry>(
				"state/odom", 10, &Server::callback_state_odom, this );
			sub_state_battery_ = nh_.subscribe<sensor_msgs::BatteryState>(
				"state/battery", 10, &Server::callback_state_battery,
				this );
			sub_state_joints_ = nh_.subscribe<sensor_msgs::JointState>(
				"state/joints", 10, &Server::callback_state_joints, this );
			sub_state_imu_ = nh_.subscribe<sensor_msgs::Imu>(
				"state/imu_data", 10, &Server::callback_state_imu, this );
			sub_state_mav_ = nh_.subscribe<mavros_msgs::State>(
				"state/mav_state", 10, &Server::callback_state_mav, this );

			pub_state_ = nh_.advertise<mantis_msgs::State>( "state", 10 );
			timer_estimator_ = nhp_.createTimer( ros::Duration( 1.0 / param_rate_ ),
				&Server::callback_estimator, this );

			ROS_INFO( "Mantis state server running!" );
		} else {
			ROS_FATAL( "State server not started, no update rate given" );
		}
	}
}

Server::~Server() {
}

void Server::callback_estimator( const ros::TimerEvent& e ) {
	// If we can get a dt, and our r vector looks valid
	if( ( e.last_real > ros::Time( 0 ) ) &&
		( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) == sensor_data_.joints.num ) ) {

		double dt = (e.current_real - e.last_real).toSec();

		//Attitude Estimation
		//TODO

		//World State Estimation
		//TODO

		//== DATA DUMP, REMOVE LATER! ==//
		g_.linear() = MDTools::quaternion_from_msg( sensor_data_.att.attitude ).toRotationMatrix();
		g_.translation() = MDTools::point_from_msg( sensor_data_.pos.position );

		bv_ = MDTools::vector_from_msg( sensor_data_.bvel.linear_vel );
		ba_ = MDTools::vector_from_msg( sensor_data_.accel.linear_acceleration );

		bw_ = MDTools::vector_from_msg( sensor_data_.gyro.angular_velocity );

		r_ = sensor_data_.joints.positions;
		rd_ = sensor_data_.joints.velocities;
		//== DATA DUMP, REMOVE LATER! ==//

		//Sensor State Estimation
		//TODO: maybe include these in the full state estimator
		voltage_ = MDTools::lpf(sensor_data_.battery.voltage, voltage_, 0.6); //TODO: Replace 0.6 with parameter!

		//Prepare state message
		mantis_msgs::State state;
		state.header.stamp = e.current_real;
		state.header.frame_id = "map";
		state.child_frame_id = "mantis_uav";
		state.configuration_stamp = p_.get(MantisParams::PARAM_TIME_CHANGE_CONFIG);

		state.pose = MDTools::pose_from_eig( g_ );
		state.twist.linear = MDTools::vector_from_eig( bv_ );
		state.twist.angular = MDTools::vector_from_eig( bw_ );
		state.accel.linear = MDTools::vector_from_eig( ba_ );
		//TODO(?): state.accel.angular = MDTools::vector_from_eig( bwa_ );

		// Some safety in case out joint configuration changes
		// This should make things wait until r_ is updated by the joiint callback
		state.r.resize( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
		state.rd.resize( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
		//state.rdd.resize( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );

		for ( int i = 0; i < p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC); i++ ) {
			state.r[i] = r_[i];
			state.rd[i] = rd_[i];
			//state.rdd[i] = rdd_[i];
		}

		state.battery_voltage = voltage_;
		state.mav_safety_disengaged = mav_ready_;	//XXX: Need to implement this properly when fuctionallity is added in mavlink
		state.mav_ready = mav_ready_;

		// Publish the final state estimate
		pub_state_.publish( state );
	} else {
		ROS_WARN_THROTTLE(10.0, "Joint state data length inconsistent joint parameters");
	}
}

void Server::update_timing_data(sensor_info_t& info, const ros::Time& stamp) {
	double rate = (stamp - info.stamp).toSec();

	info.avg_rate = MDTools::lpf(rate, info.avg_rate, 0.6);

	//TODO: Need to implement a method similar to mavel to eliminate jitter
	if( -sensor_params_.rate_deviation < ( info.avg_rate - info.exp_rate ) ) {
		info.data_valid = true;
	} else {
		info.data_valid = false;
	}

	info.stamp = stamp;
}

void Server::callback_state_odom( const nav_msgs::Odometry::ConstPtr& msg_in ) {
	update_timing_data( sensor_data_.pos.info, msg_in->header.stamp );
	sensor_data_.att.info = sensor_data_.pos.info;
	sensor_data_.bvel.info = sensor_data_.pos.info;

	sensor_data_.pos.position = msg_in->pose.pose.position;
	sensor_data_.att.attitude = msg_in->pose.pose.orientation;
	sensor_data_.bvel.linear_vel = msg_in->twist.twist.linear;

	/*
	double dt = ( msg_in->header.stamp - msg_odom_tr_ ).toSec();
	msg_odom_tr_ = msg_in->header.stamp;

	// Update pose
	Eigen::Affine3d g = MDTools::affine_from_msg( msg_in->pose.pose );
	update_g( g );

	// Update body velocities
	Eigen::Vector3d bv = MDTools::vector_from_msg( msg_in->twist.twist.linear );
	update_bv( bv );

	if ( param_use_odom_avel_ ) {
		Eigen::Vector3d bw = MDTools::vector_from_msg( msg_in->twist.twist.angular );
		update_bw( bw, dt );
	}
	*/
}

void Server::callback_state_battery( const sensor_msgs::BatteryState::ConstPtr& msg_in ) {
	// XXX: Need to do this as the straight voltage reading is too slow
	double voltage = 0.0;
	for ( int i = 0; i < msg_in->cell_voltage.size(); i++ ) {
		voltage += msg_in->cell_voltage[i];
	}

	update_timing_data( sensor_data_.battery.info, msg_in->header.stamp );
	sensor_data_.battery.voltage = voltage;
}

void Server::callback_state_imu( const sensor_msgs::Imu::ConstPtr& msg_in ) {
	update_timing_data( sensor_data_.accel.info, msg_in->header.stamp );
	sensor_data_.gyro.info = sensor_data_.accel.info;

	sensor_data_.accel.linear_acceleration = msg_in->linear_acceleration;
	sensor_data_.gyro.angular_velocity = msg_in->angular_velocity;
}

void Server::callback_state_mav(
	const mavros_msgs::State::ConstPtr& msg_in ) {

	if( msg_in->mode == "OFFBOARD" ) {
		mav_ready_ = msg_in->armed;
	} else {
		ROS_WARN_THROTTLE(10.0, "Autopilot not in OFFBOARD mode, i.e. not ready");
		mav_ready_ = false;
	}

	//mav_ready_ = msg_in->armed && ( msg_in->mode == "OFFBOARD" );
}

void Server::callback_state_joints( const sensor_msgs::JointState::ConstPtr& msg_in ) {
	//double dt = ( msg_in->header.stamp - msg_joints_tr_ ).toSec();

	Eigen::VectorXd r = Eigen::VectorXd::Zero( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
	Eigen::VectorXd rd = Eigen::VectorXd::Zero( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
	//Eigen::VectorXd rdd = Eigen::VectorXd::Zero( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );

	int jc = 0; // Seperate counter is used to increment only for dynamic joints
	for ( int i = 0; i < p_.get(MantisParams::PARAM_JOINT_NUM); i++ ) {
		dh_parameters::JointDescription joint = p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i);

		if ( joint.type != "static" ) {
			for ( int j = 0; j < msg_in->name.size(); j++ ) {
				// If we have the right joint
				if ( joint.name == msg_in->name[j] ) {
					r( jc ) = msg_in->position[j];
					rd( jc ) = msg_in->velocity[j];

					// Only update rdd if dt is acceptable
					//if ( ( dt > 0.0 ) && ( dt < 1.0 ) )
					//	rdd( jc ) = ( rd( jc ) - rd_( jc ) ) / dt; // TODO: FILTER?
				}
			}

			jc++;
		}
	}


	update_timing_data( sensor_data_.joints.info, msg_in->header.stamp );
	sensor_data_.joints.num = jc;
	sensor_data_.joints.positions = r;
	sensor_data_.joints.velocities = rd;

	//update_r( r );
	//update_rd( rd );
	// Only update rdd if dt is acceptable
	//if ( ( dt > 0.0 ) && ( dt < 1.0 ) )
	//	update_rdd( rdd );

	//msg_joints_tr_ = msg_in->header.stamp;
}

};
