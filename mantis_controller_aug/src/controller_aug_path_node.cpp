#include <ros/ros.h>
#include <controller_aug/controller_aug_path.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_aug_path");
	ControllerAugPath augp;

	ros::spin();

	return 0;
}
