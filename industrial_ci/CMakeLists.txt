cmake_minimum_required(VERSION 2.8.3)
project(industrial_ci)
find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS scripts/run_ci scripts/rerun_ci
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )
catkin_install_python(PROGRAMS scripts/run_travis
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
