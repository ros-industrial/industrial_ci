cmake_minimum_required(VERSION 2.8.3)
project(test_pylint)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS script/sample_talker.py script/sample_talker2.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
