#include <ros/ros.h>
#include <mantis_controller_mod/controller_mod.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_mod");
	ControllerMod mod;

	ros::spin();

	return 0;
}
