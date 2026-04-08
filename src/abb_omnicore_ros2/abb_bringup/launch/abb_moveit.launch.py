from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "sim", default_value="false", description="Simulation flag"
        ),
        DeclareLaunchArgument(
            "robot_type",
            default_value="gofa_crb15000_10_152",
            description="Robot name, e.g. gofa_crb15000_10/1.52",
        ),
        DeclareLaunchArgument(
            "moveit_namespace",
            default_value="",
            description="Namespace for moveit controller_manager. "
                        "If empty, uses the same namespace as abb_control (may conflict). "
                        "Recommended: set to 'moveit_' when used alongside abb_control.",
        ),
        DeclareLaunchArgument(
            "controller_manager_name",
            default_value="controller_manager",
            description="Controller manager name for spawners (e.g. 'controller_manager' or 'moveit_/controller_manager')",
        ),
    ]

    sim_arg = LaunchConfiguration("sim")
    robot_type = LaunchConfiguration("robot_type")
    moveit_namespace = LaunchConfiguration("moveit_namespace")
    controller_manager_name = LaunchConfiguration("controller_manager_name")

    moveit_config_pkg = FindPackageShare(
        [
            TextSubstitution(text="abb_"),
            robot_type,
            TextSubstitution(text="_moveit_config"),
        ]
    )

    # If moveit_namespace is non-empty, wrap the include in a PushNamespace
    # so that moveit's ros2_control_node gets its own namespace
    # The controller_manager_name must match: when namespaced, the spawner needs
    # the full path (e.g., "moveit_/controller_manager") not just "controller_manager"
    controller_manager_name = LaunchConfiguration("controller_manager_name")

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        moveit_config_pkg,
                        "launch",
                        "moveit.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "sim": sim_arg,
            "robot_type": robot_type,
            "controller_manager_name": controller_manager_name,
        }.items(),
    )

    if moveit_namespace:
        launch_entities = [
            LogInfo(msg=["MoveIt namespace: ", moveit_namespace]),
            GroupAction([
                PushRosNamespace(namespace=moveit_namespace),
                included_launch,
            ]),
        ]
    else:
        launch_entities = [
            LogInfo(msg=["Using MoveIt config package: ", moveit_config_pkg]),
            included_launch,
        ]

    return LaunchDescription(
        declared_arguments + launch_entities
    )
