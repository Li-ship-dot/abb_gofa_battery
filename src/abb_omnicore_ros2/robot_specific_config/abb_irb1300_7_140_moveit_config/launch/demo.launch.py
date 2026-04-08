from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                get_package_share_directory(
                                    "abb_irb1300_7_140_moveit_config"
                                ),
                                "launch",
                                "moveit.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments={"sim": "True"}.items(),
            )
        ]
    )
