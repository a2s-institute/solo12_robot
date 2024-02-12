from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_args(context):
    """
    Launch arguments definition using an "opaque function"
    to be able to later evaluate the launch argument at runtime.
    """

    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            "stand",
            default_value="true",
            description="Spawn stand for the robot."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="solo12",
            description="Robot name."
        )
    )

    return declared_args

def launch_setup(context):
    """
    Launch setup with access to `LaunchContext` to be able to read
    launch arguments at runtime.
    """

    gazebo_configuration = PathJoinSubstitution(
        [
            FindPackageShare("solo12_description"),
            "config",
            "gazebo.yaml"
        ]
    )

    launch_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ]
        ),
        launch_arguments={
            "extra_gazebo_args": f"--ros-args --params-file {gazebo_configuration.perform(context)}"
        }.items(),
    )

    spawn_stand = ExecuteProcess(
        cmd=[
            "xacro",
            PathJoinSubstitution(
                [
                    FindPackageShare("solo12_description"),
                    "urdf",
                    "stand.urdf"
                ]
            ),
            "|",
            "ros2", "run", "gazebo_ros", "spawn_entity.py", "-stdin", "-entity", "stand"
        ],
        output="both",
        shell=True,
        condition=IfCondition(LaunchConfiguration("stand"))
    )

    # subscribe to the `/robot_description` topic to wait for the robot definition.
    # this node spawn the robot wen
    # this node is launched only when the argument `stand` is false (just the robot in gazebo). 
    only_spawn_solo_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=['-topic', "robot_description",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.54", 
                   "-entity", "solo12"],
        condition=UnlessCondition(LaunchConfiguration("stand"))
    )


    # If `stand`, launch this node. Used with `delay_spawn_solo_robot`
    # Spawn the stand first and then the robot
    spawn_solo_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=['-topic', "robot_description",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.54", 
                   "-entity", LaunchConfiguration("robot_name")],
    )

    delay_spawn_solo_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_stand,
            on_exit=[spawn_solo_robot],
        )
    )

    return [
        launch_gazebo,
        spawn_stand,
        only_spawn_solo_robot,
        delay_spawn_solo_robot
    ]


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
