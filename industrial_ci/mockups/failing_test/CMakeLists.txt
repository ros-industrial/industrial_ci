cmake_minimum_required(VERSION 3.0.2)
project(failing_test)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/no_talker.test)
endif()

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
