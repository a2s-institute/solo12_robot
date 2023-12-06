import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config_file = PathJoinSubstitution([FindPackageShare("solo12_description"), "rviz", "solo12.rviz"])
    
    # Get URDF path
    urdf_path = os.path.join(get_package_share_directory("solo12_description"), 'urdf', 'solo12.urdf.xacro')

    robot_description = {"robot_description": xacro.process_file(urdf_path).toxml()}

    gazebo = IncludeLaunchDescription(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"))

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=['-topic', "robot_description", "-entity", "solo12"]
            )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
            gazebo,
            spawn
        ]
    )
