import os
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
import xacro
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.")
    )

    use_rviz = LaunchConfiguration("use_rviz")

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("solo12_description"),
            "rviz",
            "solo12.rviz"
        ]
    )

    # Get URDF path
    urdf_path = os.path.join(
        get_package_share_directory("solo12_description"),
        'urdf',
        'solo12.urdf.xacro'
    )

    # robot description parameter composition
    robot_description = {"robot_description": xacro.process_file(urdf_path).toxml()}

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ]
        )
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=['-topic', "robot_description", "-entity", "solo12"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )


    return LaunchDescription(
        declared_arguments + 
        [
            gazebo,
            robot_state_pub_node,
            spawn_entity_node,
            joint_state_broadcaster_spawner,
            delay_rviz
        ]
    )
