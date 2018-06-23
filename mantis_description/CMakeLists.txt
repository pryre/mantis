cmake_minimum_required(VERSION 2.8.3)
project(mantis_description)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dh_parameters
  mantis_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_param_client ${PROJECT_NAME}_state_client ${PROJECT_NAME}_solver
  CATKIN_DEPENDS dh_parameters
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_param_server
  src/mantis_description/param_server.cpp
)

add_library(${PROJECT_NAME}_state_server
  src/mantis_description/state_server.cpp
)

add_library(${PROJECT_NAME}_param_client
  src/mantis_description/param_client.cpp
)

add_library(${PROJECT_NAME}_state_client
  src/mantis_description/state_client.cpp
)

add_library(${PROJECT_NAME}_solver
  src/mantis_description/solver.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_param_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_state_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_param_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_state_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
target_link_libraries(${PROJECT_NAME}_param_server
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_param_client
  ${catkin_LIBRARIES}
)


target_link_libraries(${PROJECT_NAME}_state_server
  ${PROJECT_NAME}_param_client
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_state_client
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_solver
  ${PROJECT_NAME}_param_client
  ${PROJECT_NAME}_param_server
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_param_server
  ${PROJECT_NAME}_state_server
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  DIRECTORY
    config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY
    meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(
  DIRECTORY
    manoeuvres/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/manoeuvres
)

install(
  DIRECTORY
    scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY
    urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
