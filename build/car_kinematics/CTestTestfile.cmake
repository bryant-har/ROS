# CMake generated Testfile for 
# Source directory: /home/bjh254/homework_ws/src/cs4750_student/hw2_kinematics/car_kinematics
# Build directory: /home/bjh254/homework_ws/build/car_kinematics
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_car_kinematics_nosetests_test.kinematic_model.py "/home/bjh254/homework_ws/build/car_kinematics/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/bjh254/homework_ws/build/car_kinematics/test_results/car_kinematics/nosetests-test.kinematic_model.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/bjh254/homework_ws/build/car_kinematics/test_results/car_kinematics" "/usr/bin/nosetests3 -P --process-timeout=60 /home/bjh254/homework_ws/src/cs4750_student/hw2_kinematics/car_kinematics/test/kinematic_model.py --with-xunit --xunit-file=/home/bjh254/homework_ws/build/car_kinematics/test_results/car_kinematics/nosetests-test.kinematic_model.py.xml")
set_tests_properties(_ctest_car_kinematics_nosetests_test.kinematic_model.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/bjh254/homework_ws/src/cs4750_student/hw2_kinematics/car_kinematics/CMakeLists.txt;27;catkin_add_nosetests;/home/bjh254/homework_ws/src/cs4750_student/hw2_kinematics/car_kinematics/CMakeLists.txt;0;")
subdirs("gtest")
