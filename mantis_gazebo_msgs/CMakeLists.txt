# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(mantis_gazebo_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_service_files(FILES DoArm.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()
