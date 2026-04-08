"""Launch file for C-scan scanning system integration.

This launch file extends abb_alignment_bringup with C-scan scanning components:
- robot_state_publisher: publishes robot URDF TF (base_link -> tool0)
- abb_robot_pose: converts joint angles to Cartesian TCP pose (/egm/robot_pose)
- abb_hand_eye_calibration: provides TF base_link -> camera_optical_frame
- battery_pose_estimator: estimates battery pose from vision
- battery_alignment_controller: PBVS alignment control (alignment stage only)
- quintic_trajectory_planner: smooths joint trajectories
- move_group: provides IK services
- cscan_trajectory_generator: generates raster scan path over battery surface

C-scan scanning workflow:
1. alignment completes -> /alignment_status = true
2. User calls /cscan/start_scan to trigger scanning
3. cscan_trajectory_generator moves along battery surface grid
4. At each grid point -> publishes /cscan_grid_trigger (Int32 index)
5. External NI system subscribes to /cscan_grid_trigger, collects ultrasound data
6. Scan completes -> /cscan_status = COMPLETE

External integration options for /cscan_grid_trigger:
- ROS2 topic (simplest): ros2 topic echo /cscan_grid_trigger
- UDP socket (lower latency): cscan_trajectory_generator can be configured
  to send UDP trigger packets to NI host fixed IP:PORT
- GPIO trigger (most precise): requires ros2_gpio package, external wiring
  to NI PFI input

Execution order (via TimerAction):
0.0s: robot_state_publisher, abb_robot_pose, controller_manager, move_group
1.0s: hand_eye_calibrator, battery_detector, battery_pose_estimator
2.0s: battery_alignment_controller, quintic_trajectory_planner
3.0s: cscan_trajectory_generator <- C-scan scan node starts last
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

    # Battery detector HSV threshold arguments (tune for your battery color)
    declared_arguments.append(
        DeclareLaunchArgument('hsv_h_min', default_value='5',
            description='HSV H min for battery color detection'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('hsv_h_max', default_value='25',
            description='HSV H max for battery color detection'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('hsv_s_min', default_value='100',
            description='HSV S min for battery color detection'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('hsv_s_max', default_value='255',
            description='HSV S max for battery color detection'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('hsv_v_min', default_value='100',
            description='HSV V min for battery color detection'),
    )
    declared_arguments.append(
        DeclareLaunchArgument('hsv_v_max', default_value='255',
            description='HSV V max for battery color detection'),
    )

    # NI ultrasonic system arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'ni_ip',
            default_value='',
            description='NI ultrasonic system IP address (empty = unconfigured)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_ni',
            default_value='false',
            description='Use mock NI ultrasonic system (for testing without hardware)',
        )
    )

    # C-scan scanning arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'cscan_battery_length_m',
            default_value='0.07',
            description='Battery length (long side) in meters',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cscan_battery_width_m',
            default_value='0.025',
            description='Battery width (short side) in meters',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cscan_resolution_m',
            default_value='0.001',
            description='Grid spacing in meters (scan resolution)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cscan_speed_m_s',
            default_value='0.005',
            description='Scan speed in meters per second',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cscan_approach_height_m',
            default_value='0.05',
            description='Distance from probe to battery surface in meters',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cscan_dwell_time_s',
            default_value='0.05',
            description='Dwell time at each grid point in seconds (for NI acquisition)',
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
    ni_ip = LaunchConfiguration('ni_ip')
    use_mock_ni = LaunchConfiguration('use_mock_ni')

    # C-scan configurations
    cscan_battery_length_m = LaunchConfiguration('cscan_battery_length_m')
    cscan_battery_width_m = LaunchConfiguration('cscan_battery_width_m')
    cscan_resolution_m = LaunchConfiguration('cscan_resolution_m')
    cscan_speed_m_s = LaunchConfiguration('cscan_speed_m_s')
    cscan_approach_height_m = LaunchConfiguration('cscan_approach_height_m')
    cscan_dwell_time_s = LaunchConfiguration('cscan_dwell_time_s')

    # Get package paths
    abb_bringup_share = get_package_share_directory('abb_bringup')
    abb_vision_share = get_package_share_directory('abb_vision')

    # Build full camera_info_file path
    full_camera_info_path = PathJoinSubstitution([
        abb_vision_share,
        camera_info_file,
    ])

    # =========================================================================
    # 0.0s: robot_state_publisher, controller_manager, move_group
    # abb_robot_pose 单独延迟到 0.5s，避免与 robot_state_publisher 竞争 TF lookup
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
            'controller_manager_name': 'moveit_/moveit_simple_controller_manager',
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
            'sim': sim,
            'camera_info_file': camera_info_file,
            'use_real_camera': use_real_camera,
            'calibration_file': calibration_file,
        }.items(),
    )

    # Mock hand-eye: abb_cscan_bringup provides its own joint_to_cartesian_pose_node
    # (only when NOT using abb_hand_eye, to avoid duplicate nodes)
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
    # Uses a reasonable offset: camera mounted 0.1m above and 0.2m forward of tool0
    # This is a placeholder calibration that can be replaced with actual calibration values
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

    # mock_data_node - provides fake hardware inputs (joint_states, camera/image_raw, battery_bboxes)
    # Only starts in simulation mode (sim=true)
    mock_data_node = Node(
        package='abb_vision',
        executable='mock_data_node.py',
        name='mock_data_node',
        condition=IfCondition(sim),
        parameters=[{
            'publish_rate_joints': 30.0,
            'publish_rate_camera': 5.0,
            'publish_rate_bboxes': 2.0,
        }],
        output='screen',
    )

    # =========================================================================
    # 2.0s: battery_alignment_controller, quintic_trajectory_planner
    # =========================================================================

    # battery_alignment_controller - PBVS alignment control (alignment stage only)
    # NOTE: Do NOT start abb_vision battery_alignment_controller here - it is
    # for alignment phase only. C-scan runs after alignment completes.
    battery_alignment_controller_node = Node(
        package='abb_vision',
        executable='battery_alignment_controller',
        name='battery_alignment_controller',
        parameters=[{
            'approach_height_m': approach_height_m,
            'alignment_timeout_sec': 2.0,
            'max_retry': 3,
            'alignment_threshold_m': 0.005,
            'alignment_threshold_rad': 0.1,
            'ik_group_name': 'manipulator',
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
    # 3.0s: cscan_trajectory_generator, cscan_udp_bridge
    # =========================================================================

    # cscan_trajectory_generator - generates raster scan path over battery surface
    # Publishes /cscan_grid_trigger (Int32) at each grid point for NI system
    # Subscribes to /cscan/start_scan and /cscan/stop_scan for scan control
    cscan_node = Node(
        package='abb_vision',
        executable='cscan_trajectory_generator',
        name='cscan_trajectory_generator',
        parameters=[{
            'battery_length_m': cscan_battery_length_m,
            'battery_width_m': cscan_battery_width_m,
            'scan_resolution_m': cscan_resolution_m,
            'scan_speed_m_s': cscan_speed_m_s,
            'approach_height_m': cscan_approach_height_m,
            'dwell_time_sec': cscan_dwell_time_s,
            'ik_group_name': 'manipulator',
            'ik_timeout_sec': 2.0,
            'position_threshold_m': 0.002,
        }],
        remappings=[
            ('/battery_poses', '/battery_poses'),
            ('/joint_states', '/joint_states'),
            ('/target_joint_states', '/target_joint_states'),
            ('/cscan_grid_trigger', '/cscan_grid_trigger'),
            ('/cscan_status', '/cscan_status'),
        ],
        output='screen',
    )

    # cscan_udp_bridge - ROS2 ↔ NI UDP bridge
    # Receives /cscan_grid_trigger -> sends UDP trigger to NI
    # Receives UDP from NI -> publishes /ultrasonic_data, /ultrasonic_envelope
    cscan_udp_bridge_node = Node(
        package='abb_vision',
        executable='cscan_udp_bridge',
        name='cscan_udp_bridge',
        parameters=[{
            'ni_ip': ni_ip,
            'use_mock_ni': use_mock_ni,
            'ni_receive_port': 5000,
            'ni_send_port': 5001,
            'local_ip': '0.0.0.0',
        }],
        remappings=[
            ('/cscan_grid_trigger', '/cscan_grid_trigger'),
        ],
        output='screen',
    )

    # =========================================================================
    # 3.5s: cscan_visualizer - C-scan real-time visualization
    # =========================================================================

    # cscan_visualizer - builds and displays C-scan image in real-time
    # Subscribes to /ultrasonic_data, /cscan_grid_trigger, /cscan_status, /joint_states
    # Publishes /cscan_live_image (C-scan image), /ascan_display (A-scan waveform)
    cscan_visualizer_node = Node(
        package='abb_vision',
        executable='cscan_visualizer',
        name='cscan_visualizer',
        parameters=[{
            'battery_length_m': cscan_battery_length_m,
            'battery_width_m': cscan_battery_width_m,
            'scan_resolution_m': cscan_resolution_m,
            'image_width': 800,
            'image_height': 600,
        }],
        remappings=[
            ('/ultrasonic_data', '/ultrasonic_data'),
            ('/cscan_grid_trigger', '/cscan_grid_trigger'),
            ('/cscan_status', '/cscan_status'),
            ('/joint_states', '/joint_states'),
            ('/cscan_live_image', '/cscan_live_image'),
            ('/ascan_display', '/ascan_display'),
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

        # 0.5s: Mock hand-eye components (only when use_mock_hand_eye=true)
        # mock_data_node (sim mode only): provides fake /joint_states, /camera/image_raw, /battery_bboxes
        TimerAction(
            period=0.5,
            actions=[
                mock_robot_pose_node,
                mock_camera_tf,
                mock_data_node,
            ],
        ),

        # 1.0s: hand_eye_calibrator (only when NOT using mock hand-eye)
        # battery_detector and battery_pose_estimator ALWAYS run (receive mock data when camera unavailable)
        TimerAction(
            period=1.0,
            actions=[
                include_hand_eye,
            ],
            condition=UnlessCondition(use_mock_hand_eye),
        ),

        # 1.0s: battery_detector - only when using real camera (not mock hand-eye)
        # In mock mode, mock_data_node handles /battery_bboxes publishing
        TimerAction(
            period=1.0,
            actions=[
                battery_detector_node,
            ],
            condition=UnlessCondition(use_mock_hand_eye),
        ),

        # 1.0s: battery_pose_estimator - always runs
        # Uses /battery_bboxes from battery_detector (real camera) or mock_data_node (mock)
        TimerAction(
            period=1.0,
            actions=[
                battery_pose_estimator_node,
            ],
        ),

        # 2.0s: Alignment control and trajectory planning
        TimerAction(
            period=2.0,
            actions=[
                battery_alignment_controller_node,
                quintic_trajectory_node,
            ],
        ),

        # 3.0s: C-scan scanning + NI bridge
        TimerAction(
            period=3.0,
            actions=[
                cscan_node,
                cscan_udp_bridge_node,
            ],
        ),

        # 3.5s: C-scan visualizer (starts after UDP bridge is ready)
        TimerAction(
            period=3.5,
            actions=[
                cscan_visualizer_node,
            ],
        ),
    ])
