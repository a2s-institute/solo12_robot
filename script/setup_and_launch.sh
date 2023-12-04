#!/bin/bash

# Define the path to the solo12_robot folder relative to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SOLO12_ROBOT_DIR="$(dirname "$SCRIPT_DIR")"

# Define the desired workspace directory (one level up from solo12_robot)
WORKSPACE_DIR="$(dirname "$SOLO12_ROBOT_DIR")"

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Check if the workspace directory exists, if not create it
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    echo "Creating ROS workspace at $WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_DIR/src"
    ln -s "$SOLO12_ROBOT_DIR" "$WORKSPACE_DIR/src/"
fi

cd "$WORKSPACE_DIR"

# Build only the specified packages
colcon build --symlink-install --packages-select solo12_bringup solo12_description

# Source the local workspace
source install/setup.bash

# Define the relative paths to the URDF and SDF files
xacro_file="$SOLO12_ROBOT_DIR/solo12_description/urdf/solo12.urdf.xacro"
urdf_file="$SOLO12_ROBOT_DIR/solo12_description/urdf/solo12.urdf"
sdf_file="$SOLO12_ROBOT_DIR/solo12_description/urdf/solo12.sdf"
sdf_world_file="$SOLO12_ROBOT_DIR/solo12_description/urdf/solo12_world.sdf"

# Convert URDF.xacro to URDF
xacro $xacro_file > $urdf_file

# Convert URDF to SDF
gz sdf -p $urdf_file > $sdf_file

#unset GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SOLO12_ROBOT_DIR/solo12_description/models

# Start the model in Gazebo
gazebo --verbose $sdf_world_file

