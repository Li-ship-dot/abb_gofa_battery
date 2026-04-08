import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():

    # 1. read command line arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Simulation flag",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.125.100",
            description="Robot IP address (ABB RWS)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rws_port",
            default_value="433",
            description="RWS port (ABB RWS)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="True",
            description="Launch RViz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="gofa_crb15000_10_152",
            description="Robot name, e.g. gofa_crb15000_10_152 for GoFa CRB15000-10/1.52",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_class",
            default_value="gofa",
            description="Robot class, e.g. gofa for GoFa CRB15000",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace",
        )
    )

    sim = LaunchConfiguration("sim")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    robot_type = LaunchConfiguration("robot_type")
    robot_class = LaunchConfiguration("robot_class")

    launch_controller = os.path.join(
        get_package_share_directory("abb_bringup"),
        "launch",
        "abb_control.launch.py",
    )
    launch_moveit = os.path.join(
        get_package_share_directory("abb_bringup"),
        "launch",
        "abb_moveit.launch.py",
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_controller),
        launch_arguments={
            "description_package": [
                TextSubstitution(text="abb_"),
                robot_class,
                TextSubstitution(text="_support"),
            ],
            # "description_file": "irb1300_7_140.xacro",
            "description_file": [
                robot_type,
                TextSubstitution(text=".xacro"),
            ],
            "use_fake_hardware": sim,
            "rws_ip": LaunchConfiguration("robot_ip"),
            "rws_port": LaunchConfiguration("rws_port"),
        }.items(),
    )
    include_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_moveit),
        launch_arguments={
            "robot_type": robot_type,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    return LaunchDescription([*declared_arguments, include_controller, include_moveit])
