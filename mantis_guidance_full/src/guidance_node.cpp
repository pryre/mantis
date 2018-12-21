#include <ros/ros.h>
#include <mantis_guidance_full/manager.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_guidance_full");
	MantisGuidanceFull::Manager guide;

	ros::spin();

	return 0;
}
