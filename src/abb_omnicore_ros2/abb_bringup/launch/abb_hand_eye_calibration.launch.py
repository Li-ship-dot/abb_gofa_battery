"""Launch file for integrated hand-eye calibration workflow.

This launch file starts all components needed for hand-eye calibration:
- robot_state_publisher: publishes robot URDF TF
- abb_robot_pose: converts joint angles to Cartesian TCP pose
- realsense2_camera: RealSense D415 driver (conditional on use_real_camera)
  NOTE: D415 global shutter RGB recommended over D455 rolling shutter for ArUco tracking
- hand_eye_calibrator: main calibration node
- battery_detector: auxiliary battery detection for validation

Calibration Steps:
1. Start calibration environment:
   ros2 launch abb_bringup abb_hand_eye_calibration.launch.py
2. Confirm /egm/robot_pose has data:
   ros2 topic echo /egm/robot_pose --once
3. Move robot and fix ArUco calibration board in camera field of view
4. In rviz2, observe TF tree, confirm base_link -> tool0 exists
5. Collect >= 10 samples at different robot poses
   (建议：每个样本间机器人末端至少移动 30cm，角度变化 > 15°)
6. Call calibration service:
   ros2 service call /calibrate std_srvs/srv/Trigger "{}"
7. After successful calibration, view output Translation and Rotation
8. Verify TF:
   ros2 run tf2_ros tf2_echo base_link camera_optical_frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    declared_arguments = []

    # Robot bringup arguments (for robot_state_publisher and pose node)
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_type',
            default_value='gofa_crb15000_10_152',
            description='Robot name, e.g. gofa_crb15000_10_152 for GoFa CRB15000-10/1.52',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_class',
            default_value='gofa',
            description='Robot class, e.g. gofa for GoFa CRB15000',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.125.100',
            description='Robot IP address (ABB RWS)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rws_port',
            default_value='433',
            description='RWS port (ABB RWS)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim',
            default_value='True',
            description='Simulation flag (True for simulated robot)',
        )
    )

    # Camera-related arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_info_file',
            default_value='config/d415_camera_info.yaml.template',
            description='Path to camera intrinsic YAML (relative to abb_vision share)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_real_camera',
            default_value='false',
            description='Set to true when real camera is available',
        )
    )

    # Calibration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'aruco_marker_length',
            default_value='0.05',
            description='ArUco marker side length in meters',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'min_samples',
            default_value='10',
            description='Minimum calibration samples',
        )
    )

    # Get launch configurations
    robot_type = LaunchConfiguration('robot_type')
    robot_class = LaunchConfiguration('robot_class')
    robot_ip = LaunchConfiguration('robot_ip')
    rws_port = LaunchConfiguration('rws_port')
    sim = LaunchConfiguration('sim')
    camera_info_file = LaunchConfiguration('camera_info_file')
    use_real_camera = LaunchConfiguration('use_real_camera')
    aruco_marker_length = LaunchConfiguration('aruco_marker_length')
    min_samples = LaunchConfiguration('min_samples')

    # Get package paths
    abb_bringup_share = get_package_share_directory('abb_bringup')
    abb_vision_share = get_package_share_directory('abb_vision')

    # Build full camera_info_file path
    full_camera_info_path = PathJoinSubstitution([
        abb_vision_share,
        camera_info_file,
    ])

    # Construct description_package and description_file for xacro
    description_package = [
        TextSubstitution(text='abb_'),
        robot_class,
        TextSubstitution(text='_support'),
    ]
    description_file = [
        robot_type,
        TextSubstitution(text='.xacro'),
    ]

    # 1. robot_state_publisher - publishes robot URDF TF
    # This is part of abb_bringup's control launch
    launch_controller = os.path.join(
        abb_bringup_share,
        'launch',
        'abb_control.launch.py',
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_controller),
        condition=UnlessCondition(sim),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'use_fake_hardware': sim,
            'rws_ip': LaunchConfiguration('robot_ip'),
            'rws_port': LaunchConfiguration('rws_port'),
        }.items(),
    )

    # 2. abb_robot_pose.launch.py - joint angles to Cartesian TCP pose
    launch_robot_pose = os.path.join(
        abb_bringup_share,
        'launch',
        'abb_robot_pose.launch.py',
    )

    include_robot_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_robot_pose),
        launch_arguments={
            'target_frame': 'tool0',
            'source_frame': 'base_link',
            'source_topic': '/joint_states',
        }.items(),
    )

    # NOTE: robot_state_publisher inside include_controller is TimerAction(period=1.0)
    # abb_robot_pose must be delayed to avoid TF lookup failure at startup
    robot_pose_timer = TimerAction(
        period=1.5,
        actions=[include_robot_pose],
    )

    # For simulation mode, we still need robot_state_publisher with a valid robot description
    # Generate robot description using xacro with use_fake_hardware=true
    # Note: Command doesn't accept 'condition' parameter (that's ExecuteProcess-only).
    # TimerAction(period=1.5) already delays this, so Command runs in both sim and non-sim.
    robot_description_content_sim = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'prefix:=\'\'',
            ' ',
            'use_fake_hardware:=true',
            ' ',
            'fake_sensor_commands:=false',
            ' ',
            'rws_ip:=\'\'',
            ' ',
            'rws_port:=\'\'',
            ' ',
            'configure_via_rws:=false',
            ' ',
        ],
    )
    robot_state_publisher_sim = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content_sim, value_type=str),
            'use_sim_time': True,
        }],
        condition=IfCondition(sim),
        output='screen',
    )


    # 3. realsense2_camera.launch.py - RealSense D415 driver
    # Only start when use_real_camera=true
    # NOTE: D415 uses global shutter RGB (better for ArUco tracking than D455 rolling shutter)
    try:
        launch_realsense = os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch',
            'rs_launch.py',
        )
        include_realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_realsense),
            condition=IfCondition(use_real_camera),
            launch_arguments={
                'camera_name': 'camera',
                'device_type': 'D415',
                'align_depth': 'true',
                'enable_sync': 'true',
            }.items(),
        )
    except Exception:
        # realsense2_camera package not available
        include_realsense = ExecuteProcess(
            cmd=['echo', 'realsense2_camera package not found, cannot start camera'],
            shell=False,
            condition=IfCondition(use_real_camera),
        )

    # 4. hand_eye_calibrator - main calibration node
    hand_eye_calibrator_node = Node(
        package='abb_vision',
        executable='hand_eye_calibrator',
        name='hand_eye_calibrator',
        parameters=[{
            'aruco_dict': 'DICT_6X6_250',
            'aruco_marker_length': aruco_marker_length,
            'calibration_board_length': 0.30,  # 300mm
            'min_samples': min_samples,
            'camera_frame': 'camera_optical_frame',
            'robot_end_effector_frame': 'tool0',
            'camera_info_file': full_camera_info_path,
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/image_raw'),
        ],
        output='screen',
    )

    # 5. battery_detector - auxiliary battery detection
    battery_detector_node = Node(
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
            # HSV parameters - widened initial range for better robustness
            'hsv_h_min': 0,
            'hsv_h_max': 40,
            'hsv_s_min': 100,
            'hsv_s_max': 255,
            'hsv_v_min': 100,
            'hsv_v_max': 255,
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/image_raw'),
        ],
        output='screen',
    )

    # Print calibration steps to console
    print_calibration_steps = ExecuteProcess(
        cmd=[
            'echo', '''
===============================================
       Hand-Eye Calibration Launch
===============================================

标定步骤：
1. ros2 launch abb_bringup abb_hand_eye_calibration.launch.py
2. 确认 /egm/robot_pose 有数据: ros2 topic echo /egm/robot_pose --once
3. 移动机器人，将 ArUco 标定板固定在相机视野内
4. 在 rviz2 中观察 TF 树，确认 base_link -> tool0 存在
5. 在不同的机器人姿态下采集 >= 10 个样本
   （建议：每个样本间机器人末端至少移动 30cm，角度变化 > 15°）
6. 调用标定服务: ros2 service call /calibrate std_srvs/srv/Trigger "{}"
7. 标定成功后查看输出 Translation 和 Rotation
8. 验证 TF: ros2 run tf2_ros tf2_echo base_link camera_optical_frame

===============================================
'''
        ],
        shell=False,
        output='screen',
    )

    return LaunchDescription([
        print_calibration_steps,
        *declared_arguments,
        include_controller,
        robot_state_publisher_sim,
        robot_pose_timer,
        include_realsense,
        hand_eye_calibrator_node,
        battery_detector_node,
    ])
