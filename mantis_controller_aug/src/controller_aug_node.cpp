#include <ros/ros.h>
#include <controller_aug/controller_aug.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_aug");
	ControllerAug aug;

	ros::spin();

	return 0;
}
