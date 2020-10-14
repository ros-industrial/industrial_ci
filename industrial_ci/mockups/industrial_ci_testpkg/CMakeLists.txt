cmake_minimum_required(VERSION 2.8.3)
project(industrial_ci_testpkg)

if(FAIL_CMAKE)
  message(FATAL_ERROR "CMake was requested to fail")
endif()

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/example_ros.test)
endif()

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
