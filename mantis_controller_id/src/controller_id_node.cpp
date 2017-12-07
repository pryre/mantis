#include <ros/ros.h>
#include <controller_id/controller_id.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_id");
	ControllerID id;

	ros::spin();

	return 0;
}
