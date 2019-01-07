#include <mantis_guidance_full/manager.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "mantis_guidance_full" );
	MantisGuidanceFull::Manager guide;

	ros::spin();

	return 0;
}
