cmake_minimum_required(VERSION 2.8.3)
project(industrial_ci)
find_package(catkin REQUIRED)
catkin_package()
install(FILES check_metapackage.py example_ros.test rosdep-install.sh travis.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
