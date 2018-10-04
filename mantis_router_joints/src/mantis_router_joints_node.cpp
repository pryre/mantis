#include <ros/ros.h>
#include <mantis_router_joints/manager.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_router_joints");

	MantisRouterJoints::Manager m;

	ros::spin();

	return 0;
}
