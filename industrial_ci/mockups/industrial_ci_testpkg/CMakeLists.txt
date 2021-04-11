cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()

project(industrial_ci_testpkg)

if(FAIL_CMAKE)
  message(FATAL_ERROR "CMake was requested to fail")
endif()

find_package(catkin REQUIRED)

catkin_package()

add_library(${PROJECT_NAME}
  src/test_compiler.cpp
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/example_ros.test)
endif()

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
