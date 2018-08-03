#include <ros/ros.h>
#include <mantis_joints/mantis_joints.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_joints");
	MantisJoints joints;

	ros::spin();

	return 0;
}
