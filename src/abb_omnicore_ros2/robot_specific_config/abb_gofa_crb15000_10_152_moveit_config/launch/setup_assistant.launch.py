from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    setup_assistant = Node(
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
        output="screen",
    )

    return LaunchDescription([setup_assistant])
