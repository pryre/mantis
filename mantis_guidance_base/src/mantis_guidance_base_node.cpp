#include <mantis_guidance_base/mantis_guidance_base.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "mantis_guidance_base" );
	MantisGuidanceBase path;

	ros::spin();

	return 0;
}
