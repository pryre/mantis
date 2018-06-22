#include <ros/ros.h>
#include <mantis_description/param_server.h>
#include <mantis_description/state_server.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_description");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	//Wait for ros time before initializing the servers
	while(ros::ok() && (ros::Time::now() == ros::Time(0)) ) {
		ros::spinOnce();
		ros::Rate(5).sleep();
	}

	if(ros::ok()) {
		MantisParamServer params(&nh);

		while(ros::ok() && !params.ok() ) {
			ros::spinOnce();
			ros::Rate(5).sleep();
		}

		MantisStateServer state(&nh, &nhp);

		ros::spin();
	}

	return 0;
}
