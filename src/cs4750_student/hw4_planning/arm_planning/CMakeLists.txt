cmake_minimum_required(VERSION 3.0.2)
project(arm_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  rospy
  tf2_ros
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS
  scripts/rrt_planning
  scripts/astar_planning
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
