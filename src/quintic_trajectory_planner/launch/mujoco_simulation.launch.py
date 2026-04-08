"""
Launch file for ABB GoFa CRB15000 with MuJoCo + ros2_control

This launch file integrates:
- MuJoCo simulator as a ros2_control hardware interface
- Joint trajectory controller
- RViz for visualization
- Quintic trajectory planner
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    pkg_name = "quintic_trajectory_planner"
    pkg_share = get_package_share_directory(pkg_name)
    moveit_pkg_share = get_package_share_directory("abb_gofa_crb15000_10_152_moveit_config")
    support_pkg_share = get_package_share_directory("abb_gofa_crb15000_support")

    # Paths to config files
    mjcf_path = os.path.join(pkg_share, "config", "mujoco_description.xml")
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    # URDF xacro file
    urdf_xacro_path = os.path.join(
        moveit_pkg_share, "config", "gofa_crb15000_10_152.urdf.xacro"
    )

    # Launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Set simulation time parameter
    set_sim_time = SetParameter(name="use_sim_time", value=use_sim_time)

    # Robot description parameter
    robot_description_content = ExecuteProcess(
        cmd=["xacro", "--inorder", urdf_xacro_path],
        output="screen"
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "publish_frequency": 50.0,
        }],
        remappings=[
            ("/robot_description", "robot_description"),
        ]
    )

    # MuJoCo ros2_control node
    mujoco_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="mujoco_ros2_control",
        output="screen",
        parameters=[{
            "robot_description": "",
            "model_uri": mjcf_path,
            "update_rate": 100.0,
        }],
        remappings=[
            ("/robot_description", "robot_description"),
        ]
    )

    # Controller manager spawner
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    # RViz node
    rviz_config = os.path.join(moveit_pkg_share, "launch", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    # Joint trajectory planner node
    trajectory_planner_node = Node(
        package="quintic_trajectory_planner",
        executable="trajectory_node",
        name="quintic_trajectory_planner",
        output="screen",
        parameters=[{
            "num_joints": 6,
            "default_duration": 2.0,
            "max_velocity": 2.0,
            "max_acceleration": 5.0,
            "use_sim_time": use_sim_time
        }]
    )

    nodes = [
        set_sim_time,
        robot_state_publisher,
        TimerAction(
            period=1.0,
            actions=[
                mujoco_control_node,
                controller_spawner,
            ]
        ),
        rviz_node,
        trajectory_planner_node,
    ]

    return LaunchDescription(declared_arguments + nodes)