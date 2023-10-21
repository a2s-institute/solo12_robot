import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = "solo12_description"

    rviz_config_file = PathJoinSubstitution([FindPackageShare(package_name), "rviz", "solo12.rviz"])
    
    # Get URDF path
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'solo12.urdf')
    
    with open(urdf_path, "r") as urdf:
        robot_desc = urdf.read()

    robot_description = {"robot_description": robot_desc}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
