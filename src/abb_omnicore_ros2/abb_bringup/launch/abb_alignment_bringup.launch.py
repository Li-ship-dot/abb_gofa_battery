"""Launch file for integrated battery alignment workflow.

This launch file starts all components needed for battery alignment:
- robot_state_publisher: publishes robot URDF TF (base_link -> tool0)
- abb_robot_pose: converts joint angles to Cartesian TCP pose (/egm/robot_pose)
- abb_hand_eye_calibration: provides TF base_link -> camera_optical_frame
- battery_pose_estimator: estimates battery pose from vision
- battery_alignment_controller: PBVS alignment control
- quintic_trajectory_planner: smooths joint trajectories
- move_group: provides IK services

Execution order (via TimerAction):
0.0s: robot_state_publisher, abb_robot_pose, controller_manager, move_group
1.0s: hand_eye_calibrator, battery_detector, battery_pose_estimator
2.0s: battery_alignment_controller, quintic_trajectory_planner
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    declared_arguments = []

    # Robot bringup arguments
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
            default_value='false',
            description='Simulation flag (false for real robot)',
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='Path to hand-eye calibration result YAML (if set, loads existing calibration on startup)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hand_eye',
            default_value='false',
            description='Use mock hand-eye calibration (skip abb_hand_eye, use mock camera_optical_frame TF and /egm/robot_pose)',
        )
    )

    # Alignment-specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'approach_height_m',
            default_value='0.05',
            description='Approach height to battery in meters',
        )
    )
    # T9.4 Fix: Add missing alignment controller parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            'alignment_timeout_sec',
            default_value='2.0',
            description='IK solver timeout in seconds',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'max_retry',
            default_value='3',
            description='Maximum IK retry attempts before ERROR state',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'alignment_threshold_m',
            default_value='0.005',
            description='Position error threshold for alignment completion (meters)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'alignment_threshold_rad',
            default_value='0.1',
            description='Orientation error threshold for alignment completion (radians)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ik_group_name',
            default_value='manipulator',
            description='MoveIt IK group name',
        )
    )

    # Battery detector HSV threshold arguments (tune for your battery color)
    declared_arguments.append(
        DeclareLaunchArgument(
            'hsv_h_min', default_value='0',  # T9.1 Fix: expanded from 5 to 0 for lighting robustness
            description='HSV H min for battery color detection',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hsv_h_max', default_value='40',  # T9.1 Fix: expanded from 25 to 40 for lighting robustness
            description='HSV H max for battery color detection',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hsv_s_min', default_value='100',
            description='HSV S min for battery color detection',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hsv_s_max', default_value='255',
            description='HSV S max for battery color detection',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hsv_v_min', default_value='100',
            description='HSV V min for battery color detection',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hsv_v_max', default_value='255',
            description='HSV V max for battery color detection',
        )
    )

    # Get launch configurations
    robot_type = LaunchConfiguration('robot_type')
    robot_class = LaunchConfiguration('robot_class')
    sim = LaunchConfiguration('sim')
    camera_info_file = LaunchConfiguration('camera_info_file')
    use_real_camera = LaunchConfiguration('use_real_camera')
    calibration_file = LaunchConfiguration('calibration_file')
    use_mock_hand_eye = LaunchConfiguration('use_mock_hand_eye')
    approach_height_m = LaunchConfiguration('approach_height_m')
    hsv_h_min = LaunchConfiguration('hsv_h_min')
    hsv_h_max = LaunchConfiguration('hsv_h_max')
    hsv_s_min = LaunchConfiguration('hsv_s_min')
    hsv_s_max = LaunchConfiguration('hsv_s_max')
    hsv_v_min = LaunchConfiguration('hsv_v_min')
    hsv_v_max = LaunchConfiguration('hsv_v_max')
    robot_ip = LaunchConfiguration('robot_ip')
    rws_port = LaunchConfiguration('rws_port')
    # T9.4 Fix: Add alignment controller LaunchConfigurations
    alignment_timeout_sec = LaunchConfiguration('alignment_timeout_sec')
    max_retry = LaunchConfiguration('max_retry')
    alignment_threshold_m = LaunchConfiguration('alignment_threshold_m')
    alignment_threshold_rad = LaunchConfiguration('alignment_threshold_rad')
    ik_group_name = LaunchConfiguration('ik_group_name')

    # Get package paths
    abb_bringup_share = get_package_share_directory('abb_bringup')
    abb_vision_share = get_package_share_directory('abb_vision')

    # Build full camera_info_file path
    full_camera_info_path = PathJoinSubstitution([
        abb_vision_share,
        camera_info_file,
    ])

    # =========================================================================
    # 0.0s: robot_state_publisher, abb_robot_pose, controller_manager, move_group
    # =========================================================================

    # abb_control.launch.py - contains controller_manager + robot_state_publisher
    launch_controller = os.path.join(
        abb_bringup_share,
        'launch',
        'abb_control.launch.py',
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_controller),
        launch_arguments={
            'description_package': [
                TextSubstitution(text='abb_'),
                robot_class,
                TextSubstitution(text='_support'),
            ],
            'description_file': [
                robot_type,
                TextSubstitution(text='.xacro'),
            ],
            'use_fake_hardware': sim,
            'rws_ip': LaunchConfiguration('robot_ip'),
            'rws_port': LaunchConfiguration('rws_port'),
        }.items(),
    )

    # abb_robot_pose.launch.py - joint_to_cartesian_pose_node (for /egm/robot_pose)
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

    # abb_moveit.launch.py - move_group for IK services
    launch_moveit = os.path.join(
        abb_bringup_share,
        'launch',
        'abb_moveit.launch.py',
    )

    include_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_moveit),
        launch_arguments={
            'sim': sim,
            'robot_type': robot_type,
            'moveit_namespace': 'moveit_',
            'controller_manager_name': 'moveit_/controller_manager',
        }.items(),
    )

    # =========================================================================
    # 1.0s: hand_eye_calibrator, battery_detector, battery_pose_estimator
    # =========================================================================

    # abb_hand_eye_calibration.launch.py - provides base_link -> camera_optical_frame TF
    launch_hand_eye = os.path.join(
        abb_bringup_share,
        'launch',
        'abb_hand_eye_calibration.launch.py',
    )

    include_hand_eye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_hand_eye),
        condition=UnlessCondition(use_mock_hand_eye),
        launch_arguments={
            'robot_type': robot_type,
            'robot_class': robot_class,
            'sim': sim,  # Pass through sim mode from parent launch
            'camera_info_file': camera_info_file,
            'use_real_camera': use_real_camera,
            'calibration_file': calibration_file,
        }.items(),
    )

    # Mock hand-eye: provides joint_to_cartesian_pose_node and camera_optical_frame TF
    mock_robot_pose_node = Node(
        package='abb_rws_client',
        executable='joint_to_cartesian_pose_node',
        name='joint_to_cartesian_pose',
        condition=IfCondition(use_mock_hand_eye),
        parameters=[{
            'target_frame': 'tool0',
            'source_frame': 'base_link',
            'source_topic': '/joint_states',
        }],
        output='screen',
    )

    # Mock hand-eye: static TF base_link -> camera_optical_frame
    mock_camera_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0.1', '0.0', '0.05',   # x, y, z offset from base_link
            '0', '0', '0', '1',        # quaternion (identity)
            'base_link', 'camera_optical_frame',
        ],
        condition=IfCondition(use_mock_hand_eye),
        output='screen',
    )

    # battery_detector - detects battery in camera image, publishes bounding boxes
    battery_detector_node = Node(
        package='abb_vision',
        executable='battery_detector',
        name='battery_detector',
        parameters=[{
            'camera_info_file': full_camera_info_path,
            'debug_mode': True,
            'min_area': 5000.0,
            'max_area': 100000.0,
            'aspect_ratio_tol': 0.3,
            'battery_length_mm': 70.0,
            'battery_width_mm': 25.0,
            'hsv_h_min': hsv_h_min,
            'hsv_h_max': hsv_h_max,
            'hsv_s_min': hsv_s_min,
            'hsv_s_max': hsv_s_max,
            'hsv_v_min': hsv_v_min,
            'hsv_v_max': hsv_v_max,
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/image_raw'),
        ],
        output='screen',
    )

    # battery_pose_estimator - estimates battery pose from detection results
    battery_pose_estimator_node = Node(
        package='abb_vision',
        executable='battery_pose_estimator',
        name='battery_pose_estimator',
        parameters=[{
            'camera_info_file': full_camera_info_path,
            'battery_length_mm': 70.0,
            'battery_width_mm': 25.0,
            'camera_frame': 'camera_optical_frame',
            'base_frame': 'base_link',
        }],
        remappings=[
            ('/battery_bboxes', '/battery_bboxes'),
        ],
        output='screen',
    )

    # =========================================================================
    # 2.0s: battery_alignment_controller, quintic_trajectory_planner
    # =========================================================================

    # battery_alignment_controller - PBVS alignment control
    battery_alignment_controller_node = Node(
        package='abb_vision',
        executable='battery_alignment_controller',
        name='battery_alignment_controller',
        parameters=[{
            'approach_height_m': approach_height_m,
            # T9.4 Fix: Use LaunchConfiguration for tunable alignment parameters
            'alignment_timeout_sec': alignment_timeout_sec,
            'max_retry': max_retry,
            'alignment_threshold_m': alignment_threshold_m,
            'alignment_threshold_rad': alignment_threshold_rad,
            'ik_group_name': ik_group_name,
        }],
        remappings=[
            ('/battery_poses', '/battery_poses'),
            ('/egm/robot_pose', '/egm/robot_pose'),
            ('/joint_states', '/joint_states'),
            ('/target_joint_states', '/target_joint_states'),
        ],
        output='screen',
    )

    # quintic_trajectory_planner - smooths joint trajectories
    quintic_trajectory_node = Node(
        package='quintic_trajectory_planner',
        executable='trajectory_node',
        name='quintic_trajectory_planner',
        parameters=[{
            'num_joints': 6,
            'default_duration': 2.0,
            'num_trajectory_points': 100,
            'max_velocity': 2.0,  # 全局上限（保守）
            # Per-joint velocity limits 必须与 gofa_crb15000.ros2_control.xacro 一致
            'max_velocity_joints': [2.094, 2.09, 2.18, 3.49, 3.49, 3.49],
        }],
        remappings=[
            ('/target_joint_states', '/target_joint_states'),
            ('/current_joint_states', '/joint_states'),
            ('/joint_trajectory', '/abb_controller/joint_trajectory'),
        ],
        output='screen',
    )

    # =========================================================================
    # Build launch description with TimerAction for ordering
    # =========================================================================

    return LaunchDescription([
        *declared_arguments,

        # 0.0s: Core robot bringup
        TimerAction(
            period=0.0,
            actions=[
                include_controller,
                include_moveit,
            ],
        ),

        # 0.5s: abb_robot_pose (when NOT using mock hand-eye)
        TimerAction(
            period=0.5,
            actions=[
                include_robot_pose,
            ],
            condition=UnlessCondition(use_mock_hand_eye),
        ),

        # 0.5s: Mock hand-eye (when use_mock_hand_eye=true)
        TimerAction(
            period=0.5,
            actions=[
                mock_robot_pose_node,
                mock_camera_tf,
            ],
            condition=IfCondition(use_mock_hand_eye),
        ),

        # 1.0s: hand_eye_calibration (real camera, when NOT using mock)
        TimerAction(
            period=1.0,
            actions=[
                include_hand_eye,
            ],
            condition=UnlessCondition(use_mock_hand_eye),
        ),

        # 1.0s: Vision nodes (battery_detector, battery_pose_estimator)
        # Run ONLY when using real camera, NOT in mock hand-eye mode
        # In mock mode, mock_data_node handles /battery_bboxes publishing
        TimerAction(
            period=1.0,
            actions=[
                battery_detector_node,
                battery_pose_estimator_node,
            ],
            condition=UnlessCondition(use_mock_hand_eye),
        ),

        # 2.0s: Alignment control and trajectory planning
        TimerAction(
            period=2.0,
            actions=[
                battery_alignment_controller_node,
                quintic_trajectory_node,
            ],
        ),
    ])
