cmake_minimum_required(VERSION 2.8.3)
project(mantis_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	dh_parameters
	std_msgs
	geometry_msgs
	message_generation
)

## Declare ROS messages and services
#add_service_files(FILES
#	DoArm.srv
#)

add_message_files(FILES
	BodyInertial.msg
	Parameters.msg
	State.msg
	JointTrajectoryGoal.msg
	JointTrajectoryList.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
	dh_parameters
)

## Declare a catkin package
catkin_package(
#	DEPENDS
#	mantis_msgs_generate_messages
)
