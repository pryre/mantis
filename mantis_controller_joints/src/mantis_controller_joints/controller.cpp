/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <ros/ros.h>

#include <mantis_controller_joints/controller.h>
#include <pid_controller_lib/pidController.h>

#include <mantis_msgs/JointTrajectoryGoal.h>
#include <std_msgs/Float64.h>

#include <string>

namespace MantisControllerJoints {

Controller::Controller( const ros::NodeHandle& nh, std::string joint_name,
	double traj_timeout )
	: nh_( ros::NodeHandle( nh, joint_name ) )
	, have_reference_( false )
	, output_( 0.0 )
	, ref_pos_( 0.0 )
	, ref_vel_( 0.0 )
	, ref_update_t_( 0 )
	, ref_timeout_( traj_timeout )
	, pid_( nh_ ) {

	name_ = joint_name;

	pub_output_ = nh_.advertise<std_msgs::Float64>( "command", 10 );
	sub_reference_pos_ = nh_.subscribe<std_msgs::Float64>(
		"reference/pos", 10, &Controller::callback_reference_pos, this );
	sub_reference_traj_ = nh_.subscribe<mantis_msgs::JointTrajectoryGoal>(
		"reference/traj", 10, &Controller::callback_reference_traj, this );
}

Controller::~Controller() {
}

void Controller::callback_reference_pos(
	const std_msgs::Float64::ConstPtr& msg_in ) {
	ref_pos_ = msg_in->data;
	ref_vel_ = 0.0;

	if ( !have_reference_ || ( ref_update_t_ != ros::Time( 0 ) ) ) {
		ROS_INFO( "Joint controller %s switching to position mode", name_.c_str() );
		ref_update_t_ = ros::Time( 0 );
	}

	ref_update_t_ = ros::Time( 0 );

	have_reference_ = true;
}

void Controller::callback_reference_traj(
	const mantis_msgs::JointTrajectoryGoal::ConstPtr& msg_in ) {
	ref_pos_ = msg_in->position;
	ref_vel_ = msg_in->velocity;
	if ( !have_reference_ || ( ref_update_t_ == ros::Time( 0 ) ) ) {
		ROS_INFO( "Joint controller %s switching to trajectory mode", name_.c_str() );
	}

	ref_update_t_ = msg_in->header.stamp;

	have_reference_ = true;
}

void Controller::set_state( double position, double velocity ) {
	state_position_ = position;
	state_velocity_ = velocity;
}

void Controller::do_control( double dt ) {
	if ( have_reference_ ) {
		output_ = pid_.step( dt, ref_pos_, state_position_ );

		// If the trajectory target is within our timeout,
		//	then add in the velocity constant
		// Otherwise the trajectory goal is stale,
		//	so just treat it as a position goal
		if ( ( ros::Time::now() - ref_update_t_ ) < ref_timeout_ ) {
			output_ += ref_vel_;
		}

		std_msgs::Float64 msg_out;
		msg_out.data = output_;
		pub_output_.publish( msg_out );
	} else {
		output_ = 0.0;
		pid_.reset();
	}
}

double Controller::ref_pos( void ) {
	return ref_pos_;
}

double Controller::ref_vel( void ) {
	return ref_vel_;
}

double Controller::output( void ) {
	return output_;
}

std::string Controller::name( void ) {
	return name_;
}
}
