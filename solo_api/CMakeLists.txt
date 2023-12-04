cmake_minimum_required(VERSION 3.14)
project(solo_api)

# Add the path to the master_board_sdk library
find_package(master_board_sdk)
message("master_board_sdk include directory:" ${MASTER_BOARD_SDK}/include)

# Add the include directory of the master_board_sdk library
# include_directories(${MASTER_BOARD_SDK}/include)

# Add the library directory of the master_board_sdk library
# link_directories(${MASTER_BOARD_SDK}/lib)

# Add your source files here
set(SOURCES
    src/example.cpp
    # Add your source files here
)

# Create the executable
add_executable(solo_api ${SOURCES})

# Link the master_board_sdk library
target_link_libraries(solo_api master_board_sdk)
