/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <mantis_controller_feed/controller_feed.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "controller_feed" );
	ControllerFeed feed;

	ros::spin();

	return 0;
}
