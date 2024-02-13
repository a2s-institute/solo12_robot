cmake_minimum_required(VERSION 3.1)
project(solo12_control_interface)

# Find packages
find_package(gazebo REQUIRED)
find_package(websocketpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${WEBSOCKETPP_INCLUDE_DIR}
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})


# include_directories(${PROJECT_SOURCE_DIR}/path/to/nlohmann)
# include_directories("/path/to/websocketpp")
# # find_package(Boost REQUIRED COMPONENTS system)
# include_directories(${Boost_INCLUDE_DIR})
# target_link_libraries(your_target_name ${Boost_LIBRARIES})


# C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(solo12_control_interface 
    src/main.cpp 
    src/gazebo_interface.cpp 
    src/webserver.cpp 
    src/hardware_interface.cpp
)

# Link libraries
target_link_libraries(solo12_control_interface 
    ${GAZEBO_LIBRARIES}
    pthread
    tbb
    ${WEBSOCKETPP_LIBRARIES}
    ${NLOHMANN_JSON_LIBRARIES}
)

# Additional Gazebo and TBB libraries might be needed depending on your usage
