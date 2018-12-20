#include <ros/ros.h>
#include <mantis_guidance_base/mantis_guidance_base.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_guidance_base");
	MantisGuidanceBase path;

	ros::spin();

	return 0;
}
