# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 2.8.3)
project(mantis_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	dh_parameters
	contrail_msgs
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
	Movement.msg
#	Manoeuvre.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
	contrail_msgs
	dh_parameters
)

## Declare a catkin package
catkin_package(
#	DEPENDS
#	mantis_msgs_generate_messages
  CATKIN_DEPENDS dh_parameters contrail_msgs std_msgs geometry_msgs
)
