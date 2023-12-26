cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(RobotJointController)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(RobotJointController SHARED RobotJointController.cpp)
target_link_libraries(RobotJointController ${GAZEBO_LIBRARIES})
