#!/bin/bash

# Define the path to the solo12_robot folder relative to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SOLO12_ROBOT_DIR="$(dirname "$SCRIPT_DIR")"

# Run the Python script
python3 "$SCRIPT_DIR/script.py"

# Define the path to the Gazebo world file
sdf_world_file="$SOLO12_ROBOT_DIR/solo12_description/urdf/solo12_world.sdf"

# Export GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SOLO12_ROBOT_DIR/solo12_description/models

# Start the model in Gazebo
gazebo --verbose "$sdf_world_file" &