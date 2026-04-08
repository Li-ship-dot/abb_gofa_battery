"""
MoveIt + RViz + ros2_control Joint Simulation Launch File

This launch file brings up:
- Robot description (URDF from xacro)
- ros2_control with GenericSystem (mock hardware)
- MoveIt move_group
- RViz for visualization

This is the standard ros2_control simulation setup that can be used
with or without MuJoCo visualization.

Usage:
    ros2 launch abb_gofa_crb15000_10_152_moveit_config mujoco_demo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz")
    )
    declared_arguments.append(
        DeclareLaunchArgument("sim", default_value="true", description="Simulation mode")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                            description="Use fake hardware")
    )

    rviz_arg = LaunchConfiguration("rviz")
    sim_arg = LaunchConfiguration("sim")
    use_fake_arg = LaunchConfiguration("use_fake_hardware")

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("abb_gofa_crb15000_10_152")
        .robot_description(file_path="config/gofa_crb15000_10_152.urdf.xacro")
        .robot_description_semantic(file_path="config/gofa_crb15000_10_152.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Paths
    config_package_share = get_package_share_directory("abb_gofa_crb15000_10_152_moveit_config")

    # ros2_controllers yaml
    ros2_controllers_path = os.path.join(
        config_package_share, "config", "ros2_controllers.yaml"
    )

    # RViz config
    rviz_config = os.path.join(config_package_share, "launch", "moveit.rviz")

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        condition=IfCondition(sim_arg),
    )

    # Controller spawner
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
        condition=IfCondition(sim_arg),
    )

    abb_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "abb_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(rviz_arg),
    )

    # Static TF publisher
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Delayed controller spawner (wait for ros2_control to initialize)
    delayed_spawner = TimerAction(
        period=2.0,
        actions=[controller_spawner, abb_controller_spawner],
        condition=IfCondition(sim_arg),
    )

    return LaunchDescription([
        *declared_arguments,

        # Robot State Publisher
        robot_state_publisher,

        # Static TF
        static_tf_node,

        # ros2_control (after robot_description is available)
        ros2_control_node,

        # Delayed controller spawner
        delayed_spawner,

        # Move group
        move_group_node,

        # RViz
        rviz_node,
    ])