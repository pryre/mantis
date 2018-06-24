#include <ros/ros.h>
#include <mantis_description/param_server.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_description");

	//Wait for ros time before initializing the servers
	while(ros::ok() && (ros::Time::now() == ros::Time(0)) ) {
		ros::spinOnce();
		ros::Rate(5).sleep();
	}

	MantisParamServer params();

	ros::spin();

	return 0;
}
