# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 2.8.3)
project(mantis_state)

#set(CMAKE_CXX_FLAGS "-O2 -DEIGEN_NO_DEBUG -DNDEBUG")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dh_parameters
  mantis_msgs
  mantis_description
  mantis_params
  nav_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SensorAttParams.cfg
  cfg/SensorMiscParams.cfg
  cfg/SensorRateParams.cfg
  cfg/SensorScalingParams.cfg
  cfg/XacroTest.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_client
  CATKIN_DEPENDS mantis_params
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/mantis_state/kalman
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(${PROJECT_NAME}_server
  src/${PROJECT_NAME}/state_server.cpp
)

add_library(${PROJECT_NAME}_client
  src/${PROJECT_NAME}/state_client.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}_server
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_client
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_server
  ${catkin_LIBRARIES}
)

## XACRO Test Node
add_executable(${PROJECT_NAME}_xacro_test_node src/xacro_test_node.cpp)
add_dependencies(${PROJECT_NAME}_xacro_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_xacro_test_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#install(
#  DIRECTORY
#    config/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#)
#
#install(
#  DIRECTORY
#    launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
#
#install(
#  DIRECTORY
#    meshes/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
#)
#
#install(
#  DIRECTORY
#    manoeuvres/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/manoeuvres
#)
#
#install(
#  DIRECTORY
#    scripts/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
#)
#
#install(
#  DIRECTORY
#    urdf/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
#)
