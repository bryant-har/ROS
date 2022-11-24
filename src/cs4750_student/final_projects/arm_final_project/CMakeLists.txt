cmake_minimum_required(VERSION 3.0.2)
project(arm_final_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package()

install(
  DIRECTORY
  launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
  scripts/visualize_object
  scripts/airport
  scripts/fake_grasp
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)