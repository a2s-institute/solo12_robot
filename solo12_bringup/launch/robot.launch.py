from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_args(context):

    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start robot in simulation, otherwise launch for physical hardware."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("solo12_description"),
                    "rviz",
                    "solo12.rviz"
                ]
            ),
            description="Cofiguration file for rviz"
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="solo12",
            description="Robot name."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "stand",
            default_value="true",
            description="Spawn the robot in a stand."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "robot_interface",
            default_value="enp1s0",
            description="Hardware interface to communicate with solo12 master board."
        )
    )

    return declared_args

def launch_setup(context):

    state_publisher = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("solo12_bringup"),
                "launch",
                "state_publisher.launch.py"
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim"),
            "robot_interface": LaunchConfiguration("robot_interface")
        }.items()
    )

    sim_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("solo12_bringup"),
                "launch",
                "sim_gazebo.launch.py"
            ]
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "stand": LaunchConfiguration("stand"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    controllers = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("solo12_bringup"),
                "launch",
                "controllers.launch.py"
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim")
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("solo12_bringup"),
                "launch",
                "start_rviz.launch.py",
            ]
        ),
        launch_arguments={
            "rviz_config_file": LaunchConfiguration("rviz_config_file")
        }.items(),
    )

    return [
        state_publisher,
        sim_gazebo,
        controllers,
        rviz
    ]

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
