#include <mantis_controller_joints/spawner.h>
#include <ros/ros.h>

#include <string>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "mantis_controller_joints" );

	MantisControllerJoints::Spawner sp;

	ros::spin();

	return 0;
}
