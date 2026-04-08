from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("activate_controllers", default_value="true")
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    abb_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["abb_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(declared_arguments + [joint_state_broadcaster_spawner, abb_controller_spawner])
