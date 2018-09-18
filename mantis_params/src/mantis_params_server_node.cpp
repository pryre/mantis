#include <ros/ros.h>
#include <mantis_params/param_server.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_params");

	MantisParamServer params;

	ros::spin();

	return 0;
}
