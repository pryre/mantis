#include <mantis_controller_id/controller_id.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "controller_id" );
	ControllerID id;

	ros::spin();

	return 0;
}
