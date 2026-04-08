from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='abb_vision',
            executable='battery_detector',
            name='battery_detector',
            parameters=[{
                'debug_mode': True,
                'min_area': 5000.0,
                'max_area': 100000.0,
                'aspect_ratio_tol': 0.3,
                'battery_length_mm': 70.0,
                'battery_width_mm': 25.0,
            }],
            remappings=[
                ('/camera/image_raw', '/image_raw'),
            ],
            output='screen',
        ),
    ])
