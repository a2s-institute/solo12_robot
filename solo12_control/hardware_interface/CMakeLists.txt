cmake_minimum_required(VERSION 3.10)

# Set your project name
project(HardwareInterfaceProject)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Path to your master board SDK
set(MASTER_BOARD_SDK_PATH "/path/to/master-board/sdk")

# Include directories for your project and the Master Board SDK
include_directories(
    ${MASTER_BOARD_SDK_PATH}/include
    include
)

# Find required packages
# If you use any other packages, find them here

# Add the executable for your project
add_executable(${PROJECT_NAME}
    src/hardware_interface.cpp
    src/main.cpp
)

# Link with the Master Board SDK and any other necessary libraries
target_link_libraries(${PROJECT_NAME}
    ${MASTER_BOARD_SDK_PATH}/lib/libmaster_board_sdk.so
    pthread # Often required for threading
)
