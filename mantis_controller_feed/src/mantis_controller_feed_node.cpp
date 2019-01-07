#include <mantis_controller_feed/controller_feed.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "controller_feed" );
	ControllerFeed feed;

	ros::spin();

	return 0;
}
