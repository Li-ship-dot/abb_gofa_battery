"""Launch file for joint_to_cartesian_pose node.

This node publishes the TCP pose in base_link frame to /egm/robot_pose,
which is required by hand_eye_calibrator for eye-in-hand calibration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    declare_target_frame = DeclareLaunchArgument(
        'target_frame',
        default_value='tool0',
        description='TCP frame name (tool0 or tool_center)',
    )
    declare_source_frame = DeclareLaunchArgument(
        'source_frame',
        default_value='base_link',
        description='Robot base frame name')
    declare_source_topic = DeclareLaunchArgument(
        'source_topic',
        default_value='/joint_states',
        description='Joint state topic source')

    target_frame = LaunchConfiguration('target_frame')
    source_frame = LaunchConfiguration('source_frame')
    source_topic = LaunchConfiguration('source_topic')

    # Joint to Cartesian pose node
    joint_to_cartesian_pose_node = Node(
        package='abb_rws_client',
        executable='joint_to_cartesian_pose_node',
        name='joint_to_cartesian_pose',
        parameters=[{
            'target_frame': target_frame,
            'source_frame': source_frame,
            'source_topic': source_topic,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_target_frame,
        declare_source_frame,
        declare_source_topic,
        joint_to_cartesian_pose_node,
    ])