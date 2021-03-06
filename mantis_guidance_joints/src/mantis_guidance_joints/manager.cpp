/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

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

	routers_.resize( p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC) );
	spawn_stamp_ = p_.get(MantisParams::PARAM_TIME_CHANGE_CONFIG);

	int cc = 0;
	for ( int i = 0; i < p_.get(MantisParams::PARAM_JOINT_NUM); i++ ) {
		if ( p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).type != "static" ) {
			routers_[cc] = new MantisGuidanceJoints::Joint( nhp_, p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).name );
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
	if ( p_.get(MantisParams::PARAM_TIME_CHANGE_CONFIG) != spawn_stamp_ ) {
		ROS_WARN( "Reconfiguring guidance configuration" );
		configure_routers();
	}

	// For all routers
	for ( int i = 0; i < routers_.size(); i++ )
		routers_[i]->update( e.current_real );
}
}
