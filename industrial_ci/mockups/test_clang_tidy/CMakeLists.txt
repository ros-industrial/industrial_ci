cmake_minimum_required(VERSION 3.0.2)
project(test_clang_tidy)

find_package(catkin REQUIRED)

catkin_package()

add_library(${PROJECT_NAME}
  src/test_clang_tidy.cpp
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
