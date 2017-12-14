#include <ros/ros.h>
#include <forward_kinematics/forward_kinematics.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "forward_kinematics");
	ForwardKinematics fk;

	ros::spin();

	return 0;
}
