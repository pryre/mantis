#include <mantis_guidance_joints/manager.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "mantis_guidance_joints" );

	MantisGuidanceJoints::Manager m;

	ros::spin();

	return 0;
}
