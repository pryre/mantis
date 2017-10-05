#include <ros/ros.h>
#include <interface_dynamixel/interface_dynamixel.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "interface_dynamixel");
	InterfaceDynamixel id;

	ros::spin();

	return 0;
}
