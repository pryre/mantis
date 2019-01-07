#include <mantis_controller_acro/controller_acro.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "controller_acro" );
	ControllerAcro acro;

	ros::spin();

	return 0;
}
