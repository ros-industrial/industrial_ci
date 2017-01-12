cmake_minimum_required(VERSION 2.8.3)
project(industrial_ci)
find_package(catkin REQUIRED rostest)
catkin_package()
install(FILES check_metapackage.py ci_main.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
