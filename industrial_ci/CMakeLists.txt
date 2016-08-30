cmake_minimum_required(VERSION 2.8.3)
project(industrial_ci)
find_package(catkin REQUIRED rostest)
catkin_package()
install(FILES check_metapackage.py ci_main.sh rosdep-install.sh test/example_ros.test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  add_rostest(test/example_ros.test)
endif()
