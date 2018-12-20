#include <ros/ros.h>
#include <mantis_guidance_full/router.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_guidance_full");
	MantisGuidanceFull::Router router;

	ros::spin();

	return 0;
}
