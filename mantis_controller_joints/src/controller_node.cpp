#include <ros/ros.h>
#include <spawner/spawner.h>

#include <string>

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_joints");

	std::vector<std::string> c_names;

	for(int i=0; i<(argc - 1); i++)
		c_names.push_back(std::string(argv[i+1]));

	Spawner sp(c_names);

	ros::spin();

	return 0;
}
