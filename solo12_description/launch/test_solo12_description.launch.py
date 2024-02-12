import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.utilities import perform_substitutions

def launch_args(context):

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        )
    )

    return declared_arguments


def launch_setup(context):

    use_sim_time = LaunchConfiguration("use_sim_time")

    package_name = "solo12_description"

    rviz_config_file = PathJoinSubstitution([FindPackageShare(package_name), "rviz", "solo12.rviz"])
    
    # Get URDF path
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'solo12.urdf.xacro')

    robot_description = {"robot_description": xacro.process_file(urdf_path).toxml()}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    return [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
