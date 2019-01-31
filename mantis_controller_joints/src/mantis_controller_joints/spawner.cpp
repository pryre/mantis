#include <ros/ros.h>

#include <mantis_controller_joints/controller.h>
#include <mantis_controller_joints/spawner.h>

#include <vector>

namespace MantisControllerJoints {

Spawner::Spawner()
	: nh_()
	, nhp_( "~" )
	, p_( nh_ )
	, s_( nh_, p_ )
	, spawn_stamp_( 0 )
	, traj_timeout_( 0.1 )
	, param_update_rate_( 20.0 ) {

	nhp_.param( "trajectory_reference_timeout", traj_timeout_, traj_timeout_ );
	nhp_.param( "update_rate", param_update_rate_, param_update_rate_ );

	ROS_INFO(
		"Joint position controller started, waiting for param definitions..." );

	if ( p_.wait_for_params() ) {
		configure_controllers();
		timer_ = nhp_.createTimer( ros::Duration( 1.0 / param_update_rate_ ),
			&Spawner::callback_timer, this );
	} else {
		ROS_ERROR( "No controllers specified" );
		ros::shutdown();
	}
}

Spawner::~Spawner() {
	remove_controllers();
}

void Spawner::configure_controllers( void ) {
	remove_controllers();

	controllers_.resize( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
	spawn_stamp_ = p_.get(MantisParams::PARAM_TIME_CHANGE_CONFIG);

	int cc = 0;
	for ( int i = 0; i < p_.get(MantisParams::PARAM_JOINT_NUM); i++ ) {
		if ( p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).type != "static" ) {
			controllers_[cc] = new MantisControllerJoints::Controller(
				nhp_, p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).name, traj_timeout_ );
			cc++;
		}
	}

	ROS_INFO_STREAM( "Spawned " << controllers_.size() << " joint controllers" );
}

void Spawner::remove_controllers( void ) {
	for ( int i = 0; i < controllers_.size(); i++ ) {
		delete controllers_[i];
	}
}

void Spawner::callback_timer( const ros::TimerEvent& e ) {
	double dt = ( e.current_real - e.last_real ).toSec();

	// Do the control as long as the state is ok, and our dt is reasonable
	if ( s_.ok() && ( dt < 1.0 ) ) {
		// Sanity check our configuration
		if ( s_.state_configuration_stamp() != spawn_stamp_ ) {
			ROS_WARN( "Reconfiguring joint controllers" );
			configure_controllers();
		}

		// For all controllers
		for ( int i = 0; i < controllers_.size(); i++ ) {
			controllers_[i]->set_state( s_.r()[i], s_.rd()[i] );
			controllers_[i]->do_control( dt );
		}
	}
}
}
