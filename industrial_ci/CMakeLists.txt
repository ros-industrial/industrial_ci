cmake_minimum_required(VERSION 2.8.3)
project(industrial_ci)

if("$ENV{ROS_VERSION}" EQUAL "2")
    cmake_policy(VERSION 3.5)

    find_package(ament_cmake REQUIRED)
    ament_package()

    set(${PROJECT_NAME}_BIN_DESTINATION lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_SHARE_DESTINATION share/${PROJECT_NAME})
    install(PROGRAMS scripts/run_travis DESTINATION ${${PROJECT_NAME}_BIN_DESTINATION})
else()
    find_package(catkin REQUIRED)
    catkin_package()

    set(${PROJECT_NAME}_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(${PROJECT_NAME}_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    catkin_install_python(PROGRAMS scripts/run_travis DESTINATION ${${PROJECT_NAME}_BIN_DESTINATION})
endif()

install(PROGRAMS scripts/run_ci scripts/rerun_ci
    DESTINATION ${${PROJECT_NAME}_BIN_DESTINATION}
)

install(DIRECTORY src
    DESTINATION ${${PROJECT_NAME}_SHARE_DESTINATION}
)
