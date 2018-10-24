#include <ros/ros.h>
#include <mantis_router_full/router.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_router_full");
	MantisRouterFull::Router router;

	ros::spin();

	return 0;
}
