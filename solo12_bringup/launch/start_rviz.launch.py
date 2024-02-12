from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_args(context):

    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            "rviz_config_file"
        )
    )

    return

def launch_setup(context):

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    return [
        rviz_node
    ]

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
