from launch import LaunchDescription, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    camera_info_file = LaunchConfiguration('camera_info_file', default='config/d415_camera_info.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_info_file',
            default_value=camera_info_file,
            description='Path to camera intrinsics YAML file'
        ),
        Node(
            package='abb_vision',
            executable='hand_eye_calibrator',
            name='hand_eye_calibrator',
            parameters=[{
                'aruco_dict': 'DICT_6X6_250',
                'aruco_marker_length': 0.05,  # 50mm
                'calibration_board_length': 0.30,  # 300mm
                'min_samples': 10,
                'camera_frame': 'camera_optical_frame',
                'robot_end_effector_frame': 'tool0',
                'camera_info_file': camera_info_file,
            }],
            remappings=[
                ('/camera/image_raw', '/image_raw'),
                ('/robot_end_effector_pose', '/egm/robot_pose'),
            ],
            output='screen',
        ),
    ])
