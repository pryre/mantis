#include <ros/ros.h>
#include <mantis_paths/paths.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "paths");
	Paths p;

	ros::spin();

	return 0;
}
