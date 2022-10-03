# CMake generated Testfile for 
# Source directory: /home/bjh254/homework_ws/src/cs4750_student/cs4750
# Build directory: /home/bjh254/homework_ws/build/cs4750
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_cs4750_roslaunch-check_launch "/home/bjh254/homework_ws/build/cs4750/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/bjh254/homework_ws/build/cs4750/test_results/cs4750/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/bjh254/homework_ws/build/cs4750/test_results/cs4750" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/bjh254/homework_ws/build/cs4750/test_results/cs4750/roslaunch-check_launch.xml\" \"/home/bjh254/homework_ws/src/cs4750_student/cs4750/launch\" ")
set_tests_properties(_ctest_cs4750_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/bjh254/homework_ws/src/cs4750_student/cs4750/CMakeLists.txt;20;roslaunch_add_file_check;/home/bjh254/homework_ws/src/cs4750_student/cs4750/CMakeLists.txt;0;")
subdirs("gtest")
