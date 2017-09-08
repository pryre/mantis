#include <ros/ros.h>
#include <spawner/spawner.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_servo");
	Spawner sp;

	ros::spin();

	return 0;
}
