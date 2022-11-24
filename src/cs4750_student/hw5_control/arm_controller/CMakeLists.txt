cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)

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

catkin_python_setup()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS
  scripts/run_pid
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  # find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)

  # catkin_add_nosetests tests Python code directly

  catkin_add_nosetests(test/pid.py)
  # add_rostest tests the ROS integrations

endif()
