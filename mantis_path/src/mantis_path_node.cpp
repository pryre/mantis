#include <ros/ros.h>
#include <mantis_path/mantis_path.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_path");
	MantisPath path;

	ros::spin();

	return 0;
}
