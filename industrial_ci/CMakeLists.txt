cmake_minimum_required(VERSION 2.8.12)
project(industrial_ci)

if("$ENV{ROS_VERSION}" EQUAL "2")
    cmake_policy(VERSION 3.5)

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
    ament_package()

    set(${PROJECT_NAME}_BIN_DESTINATION lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_SHARE_DESTINATION share/${PROJECT_NAME})
elseif("$ENV{ROS_VERSION}" EQUAL "1")
    find_package(catkin REQUIRED)
    catkin_python_setup()
    catkin_package()

    set(${PROJECT_NAME}_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(${PROJECT_NAME}_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
    message(FATAL_ERROR "ROS_VERSION is neither 1 nor 2")
endif()

install(PROGRAMS scripts/run_ci scripts/rerun_ci scripts/run_travis
    DESTINATION ${${PROJECT_NAME}_BIN_DESTINATION}
)

install(DIRECTORY src
    DESTINATION ${${PROJECT_NAME}_SHARE_DESTINATION}
)
