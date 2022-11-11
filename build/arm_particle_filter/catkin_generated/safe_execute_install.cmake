execute_process(COMMAND "/home/bjh254/homework_ws/build/arm_particle_filter/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bjh254/homework_ws/build/arm_particle_filter/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
