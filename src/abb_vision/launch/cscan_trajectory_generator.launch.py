from launch import LaunchDescription, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Declare parameters
    battery_length_m = LaunchConfiguration('battery_length_m', default='0.07')
    battery_width_m = LaunchConfiguration('battery_width_m', default='0.025')
    scan_resolution_m = LaunchConfiguration('scan_resolution_m', default='0.001')
    scan_speed_m_s = LaunchConfiguration('scan_speed_m_s', default='0.01')
    approach_height_m = LaunchConfiguration('approach_height_m', default='0.05')
    dwell_time_sec = LaunchConfiguration('dwell_time_sec', default='0.05')
    ik_group_name = LaunchConfiguration('ik_group_name', default='manipulator')
    ik_timeout_sec = LaunchConfiguration('ik_timeout_sec', default='2.0')
    position_threshold_m = LaunchConfiguration('position_threshold_m', default='0.002')

    return LaunchDescription([
        # Parameter declarations
        DeclareLaunchArgument(
            'battery_length_m',
            default_value=battery_length_m,
            description='Battery length (long side) in meters'
        ),
        DeclareLaunchArgument(
            'battery_width_m',
            default_value=battery_width_m,
            description='Battery width (short side) in meters'
        ),
        DeclareLaunchArgument(
            'scan_resolution_m',
            default_value=scan_resolution_m,
            description='Grid spacing in meters'
        ),
        DeclareLaunchArgument(
            'scan_speed_m_s',
            default_value=scan_speed_m_s,
            description='Scan speed in meters per second'
        ),
        DeclareLaunchArgument(
            'approach_height_m',
            default_value=approach_height_m,
            description='Distance from probe to battery surface in meters'
        ),
        DeclareLaunchArgument(
            'dwell_time_sec',
            default_value=dwell_time_sec,
            description='Dwell time at each grid point in seconds'
        ),
        DeclareLaunchArgument(
            'ik_group_name',
            default_value=ik_group_name,
            description='MoveIt IK group name'
        ),
        DeclareLaunchArgument(
            'ik_timeout_sec',
            default_value=ik_timeout_sec,
            description='IK solver timeout in seconds'
        ),
        DeclareLaunchArgument(
            'position_threshold_m',
            default_value=position_threshold_m,
            description='Position error threshold in meters'
        ),

        # Node
        Node(
            package='abb_vision',
            executable='cscan_trajectory_generator',
            name='cscan_trajectory_generator',
            parameters=[{
                'battery_length_m': PythonExpression(['float(', battery_length_m, ')']),
                'battery_width_m': PythonExpression(['float(', battery_width_m, ')']),
                'scan_resolution_m': PythonExpression(['float(', scan_resolution_m, ')']),
                'scan_speed_m_s': PythonExpression(['float(', scan_speed_m_s, ')']),
                'approach_height_m': PythonExpression(['float(', approach_height_m, ')']),
                'dwell_time_sec': PythonExpression(['float(', dwell_time_sec, ')']),
                'ik_group_name': ik_group_name,
                'ik_timeout_sec': PythonExpression(['float(', ik_timeout_sec, ')']),
                'position_threshold_m': PythonExpression(['float(', position_threshold_m, ')']),
            }],
            remappings=[
                ('/battery_poses', '/battery_poses'),
                ('/joint_states', '/joint_states'),
                ('/target_joint_states', '/target_joint_states'),
                ('/cscan_grid_trigger', '/cscan_grid_trigger'),
                ('/cscan_status', '/cscan_status'),
            ],
            output='screen',
        ),
    ])
