#include <ros/ros.h>
#include <mantis_router_joints/mantis_router_joints.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_router_joints");
	MantisRouterJoints joints;

	ros::spin();

	return 0;
}
