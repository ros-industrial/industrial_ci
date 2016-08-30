cmake_minimum_required(VERSION 2.8.3)
project(industrial_ci_testpkg)

find_package(catkin REQUIRED COMPONENTS rospy_tutorials rostest)

catkin_package()

# Missing install target on purpose to confirm that industrial_ci fails.
