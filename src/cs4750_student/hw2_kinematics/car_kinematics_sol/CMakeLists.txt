cmake_minimum_required(VERSION 3.0.2)
project(car_kinematics)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
  scripts/make_rollout_plot
  scripts/make_motion_model_plot
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  # catkin_add_nosetests tests Python code directly

  # tests
  catkin_add_nosetests(test/kinematic_model.py)

endif()
