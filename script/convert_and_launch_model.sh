#!/bin/bash

# Source ROS 2 Humble and local workspace setup
source /opt/ros/humble/setup.bash
source ~/sdp_ws/install/setup.bash

# Define the relative paths to the URDF and SDF files
xacro_file="~/sdp_ws/src/solo12_robot/solo12_description/urdf/solo12.urdf.xacro"
urdf_file="~/sdp_ws/src/solo12_robot/solo12_description/urdf/solo12.urdf"
sdf_file="~/sdp_ws/src/solo12_robot/solo12_description/urdf/solo12.sdf"

# Convert URDF.xacro to URDF
xacro $xacro_file > $urdf_file

# Convert URDF to SDF
gz sdf -p $urdf_file > $sdf_file

# Start the model in Gazebo
gazebo $sdf_file

