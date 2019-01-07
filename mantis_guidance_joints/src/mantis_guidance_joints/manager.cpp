#include <ros/ros.h>

#include <mantis_guidance_joints/joint.h>
#include <mantis_guidance_joints/manager.h>

#include <vector>

namespace MantisGuidanceJoints {

Manager::Manager()
	: nh_()
	, nhp_( "~" )
	, p_( nh_ )
	, spawn_stamp_( 0 )
	, param_update_rate_( 20.0 ) {

	nhp_.param( "update_rate", param_update_rate_, param_update_rate_ );

	ROS_INFO( "Joint guidance started, waiting for param definitions..." );

	if ( p_.wait_for_params() ) {
		configure_routers();
		timer_ = nhp_.createTimer( ros::Duration( 1.0 / param_update_rate_ ),
			&Manager::callback_timer, this );
	} else {
		ROS_ERROR( "No joints specified" );
		ros::shutdown();
	}
}

Manager::~Manager() {
	remove_routers();
}

void Manager::configure_routers( void ) {
	remove_routers();

	routers_.resize( p_.get_dynamic_joint_num() );
	spawn_stamp_ = p_.time_configuration_change();

	int cc = 0;
	for ( int i = 0; i < p_.get_joint_num(); i++ ) {
		if ( p_.joint( i ).type != "static" ) {
			routers_[cc] = new MantisGuidanceJoints::Joint( nhp_, p_.joint( i ).name );
			cc++;
		}
	}

	ROS_INFO_STREAM( "Spawned guidance for " << routers_.size() << " joints" );
}

void Manager::remove_routers( void ) {
	for ( int i = 0; i < routers_.size(); i++ ) {
		delete routers_[i];
	}
}

void Manager::callback_timer( const ros::TimerEvent& e ) {
	if ( p_.time_configuration_change() != spawn_stamp_ ) {
		ROS_WARN( "Reconfiguring guidance configuration" );
		configure_routers();
	}

	// For all routers
	for ( int i = 0; i < routers_.size(); i++ )
		routers_[i]->update( e.current_real );
}
}
