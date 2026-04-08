from launch import LaunchDescription, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression

def generate_launch_description():
    camera_info_file = LaunchConfiguration('camera_info_file', default='config/d415_camera_info.yaml')
    battery_length_mm = LaunchConfiguration('battery_length_mm', default='70.0')
    battery_width_mm = LaunchConfiguration('battery_width_mm', default='25.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_info_file',
            default_value=camera_info_file,
            description='Path to camera intrinsics YAML file'
        ),
        DeclareLaunchArgument(
            'battery_length_mm',
            default_value=battery_length_mm,
            description='Battery length in millimeters'
        ),
        DeclareLaunchArgument(
            'battery_width_mm',
            default_value=battery_width_mm,
            description='Battery width in millimeters'
        ),
        Node(
            package='abb_vision',
            executable='battery_pose_estimator',
            name='battery_pose_estimator',
            parameters=[{
                'camera_info_file': camera_info_file,
                'battery_length_mm': PythonExpression(['float(', battery_length_mm, ')']),
                'battery_width_mm': PythonExpression(['float(', battery_width_mm, ')']),
                'camera_frame': 'camera_optical_frame',
                'base_frame': 'base_link',
            }],
            remappings=[
                ('/battery_bboxes', '/battery_bboxes'),  # 保持默认
            ],
            output='screen',
        ),
    ])
