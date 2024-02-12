from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("solo12_description"),
            "config",
            "solo12_controllers.yaml"
        ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers
        ],
        output="both",
        emulate_tty=True
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    velocity_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    position_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return [
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        # velocity_controllers_spawner,
        position_controllers_spawner
    ]


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
