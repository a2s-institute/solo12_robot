cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(DelayedLoading)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(DelayedLoading SHARED DelayedLoading.cpp)
target_link_libraries(DelayedLoading ${GAZEBO_LIBRARIES})
