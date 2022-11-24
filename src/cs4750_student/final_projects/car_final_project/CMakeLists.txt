cmake_minimum_required(VERSION 3.0.2)
project(car_final_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
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
  scripts/planner
  scripts/surveillance_bot
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  # find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)

endif()
