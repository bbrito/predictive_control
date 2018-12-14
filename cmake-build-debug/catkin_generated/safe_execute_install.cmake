execute_process(COMMAND "/home/demo/ros/catkin_ws/src/lmpcc/cmake-build-debug/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/demo/ros/catkin_ws/src/lmpcc/cmake-build-debug/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
