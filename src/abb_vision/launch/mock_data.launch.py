"""Launch file for mock data generation during simulation testing.

Provides fake hardware inputs to drive the full software chain without real hardware:
  - /joint_states: oscillating 6-joint state
  - /camera/image_raw: orange rectangle image
  - /battery_bboxes: 4-corner battery detection

Usage:
    ros2 launch abb_vision mock_data.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='abb_vision',
            executable='mock_data_node.py',
            name='mock_data_node',
            output='screen',
            parameters=[{
                'publish_rate_joints': 30.0,
                'publish_rate_camera': 5.0,
                'publish_rate_bboxes': 2.0,
            }],
        ),
    ])
