# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/build

# Include any dependencies generated for this target.
include CMakeFiles/solo_api.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/solo_api.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/solo_api.dir/flags.make

CMakeFiles/solo_api.dir/src/example.cpp.o: CMakeFiles/solo_api.dir/flags.make
CMakeFiles/solo_api.dir/src/example.cpp.o: ../src/example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/solo_api.dir/src/example.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/solo_api.dir/src/example.cpp.o -c /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/src/example.cpp

CMakeFiles/solo_api.dir/src/example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/solo_api.dir/src/example.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/src/example.cpp > CMakeFiles/solo_api.dir/src/example.cpp.i

CMakeFiles/solo_api.dir/src/example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/solo_api.dir/src/example.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/src/example.cpp -o CMakeFiles/solo_api.dir/src/example.cpp.s

# Object files for target solo_api
solo_api_OBJECTS = \
"CMakeFiles/solo_api.dir/src/example.cpp.o"

# External object files for target solo_api
solo_api_EXTERNAL_OBJECTS =

solo_api: CMakeFiles/solo_api.dir/src/example.cpp.o
solo_api: CMakeFiles/solo_api.dir/build.make
solo_api: CMakeFiles/solo_api.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable solo_api"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/solo_api.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/solo_api.dir/build: solo_api

.PHONY : CMakeFiles/solo_api.dir/build

CMakeFiles/solo_api.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/solo_api.dir/cmake_clean.cmake
.PHONY : CMakeFiles/solo_api.dir/clean

CMakeFiles/solo_api.dir/depend:
	cd /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/build /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/build /home/akhilan/Downloads/2_SDP/git_ws_SDP/solo12_robot/solo_api/build/CMakeFiles/solo_api.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/solo_api.dir/depend
