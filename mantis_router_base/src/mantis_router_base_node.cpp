#include <ros/ros.h>
#include <mantis_router_base/mantis_router_base.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_router_base");
	MantisRouterBase path;

	ros::spin();

	return 0;
}
