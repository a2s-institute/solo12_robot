import xacro
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_args(context):

    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        )
    )

    return declared_args

def launch_setup(context):

    robot_description_content = xacro.process_file(
        PathJoinSubstitution(
            [
                FindPackageShare("solo12_description"),
                "urdf",
                "solo12.urdf.xacro"
            ]
        ).perform(context),
        mappings={
            "use_sim_time": LaunchConfiguration("use_sim_time").perform(context)
        }
    ).toxml()

    robot_state_publiser_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return [
        robot_state_publiser_node
    ]

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
