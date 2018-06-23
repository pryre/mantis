#include <ros/ros.h>
#include <mantis_controller_aug/controller_aug_lite.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_aug");
	ControllerAugLite augl;

	ros::spin();

	return 0;
}
