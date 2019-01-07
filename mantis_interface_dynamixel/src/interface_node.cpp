#include <interface_dynamixel/interface_dynamixel.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "interface_dynamixel" );
	InterfaceDynamixel id;

	ros::spin();

	return 0;
}
