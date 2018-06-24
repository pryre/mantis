#include <ros/ros.h>
#include <mantis_state/state_server.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_state");

	MantisStateServer state;

	ros::spin();

	return 0;
}
