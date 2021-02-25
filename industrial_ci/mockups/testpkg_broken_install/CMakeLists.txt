cmake_minimum_required(VERSION 3.0.2)
project(testpkg_broken_install)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

# Install script folder but not test folder (https://github.com/ros-industrial/industrial_ci/pull/177)
install(DIRECTORY script DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

# Install something missing, to confirm that the install job fails while non-install job succeeds.   
# Suggested at https://github.com/ros-industrial/industrial_ci/pull/177#issuecomment-301688167
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
