#include <ros/ros.h>
#include <mantis_description/param_server.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_param");

	MantisParamServer params;

	ros::spin();

	return 0;
}
