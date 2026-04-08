from launch import LaunchDescription, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Declare parameters
    battery_length_m = LaunchConfiguration('battery_length_m', default='0.07')
    battery_width_m = LaunchConfiguration('battery_width_m', default='0.025')
    scan_resolution_m = LaunchConfiguration('scan_resolution_m', default='0.001')
    image_width = LaunchConfiguration('image_width', default='800')
    image_height = LaunchConfiguration('image_height', default='600')

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
            'image_width',
            default_value=image_width,
            description='C-scan image display width in pixels'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value=image_height,
            description='C-scan image display height in pixels'
        ),

        # Node
        Node(
            package='abb_vision',
            executable='cscan_visualizer',
            name='cscan_visualizer',
            parameters=[{
                'battery_length_m': PythonExpression(['float(', battery_length_m, ')']),
                'battery_width_m': PythonExpression(['float(', battery_width_m, ')']),
                'scan_resolution_m': PythonExpression(['float(', scan_resolution_m, ')']),
                'image_width': PythonExpression(['int(', image_width, ')']),
                'image_height': PythonExpression(['int(', image_height, ')']),
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
        ),
    ])