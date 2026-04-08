from launch import LaunchDescription, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression


def generate_launch_description():
    # NI parameters
    ni_ip = LaunchConfiguration('ni_ip', default='')
    use_mock_ni = LaunchConfiguration('use_mock_ni', default='false')
    ni_receive_port = LaunchConfiguration('ni_receive_port', default='5000')
    ni_send_port = LaunchConfiguration('ni_send_port', default='5001')
    local_ip = LaunchConfiguration('local_ip', default='0.0.0.0')

    return LaunchDescription([
        # NI IP address
        DeclareLaunchArgument(
            'ni_ip',
            default_value=ni_ip,
            description='NI host IP address (empty = unconfigured, use use_mock_ni:=true for testing)'
        ),

        # Use mock NI system (for testing without hardware)
        DeclareLaunchArgument(
            'use_mock_ni',
            default_value=use_mock_ni,
            description='Use mock NI ultrasonic system for testing without hardware'
        ),

        # Port for receiving NI ultrasonic data
        DeclareLaunchArgument(
            'ni_receive_port',
            default_value=ni_receive_port,
            description='Local port to receive NI UDP data'
        ),

        # Port for sending trigger to NI
        DeclareLaunchArgument(
            'ni_send_port',
            default_value=ni_send_port,
            description='NI UDP port for sending triggers'
        ),

        # Local listening address
        DeclareLaunchArgument(
            'local_ip',
            default_value=local_ip,
            description='Local IP address to bind for receiving'
        ),

        # C-scan UDP Bridge node
        Node(
            package='abb_vision',
            executable='cscan_udp_bridge',
            name='cscan_udp_bridge',
            parameters=[{
                'ni_ip': ni_ip,
                'use_mock_ni': PythonExpression(['use_mock_ni in ["true", "True"]']),
                'ni_receive_port': PythonExpression(['int(', ni_receive_port, ')']),
                'ni_send_port': PythonExpression(['int(', ni_send_port, ')']),
                'local_ip': local_ip,
            }],
            remappings=[
                ('/cscan_grid_trigger', '/cscan_grid_trigger'),
                ('/ultrasonic_data', '/ultrasonic_data'),
                ('/ultrasonic_envelope', '/ultrasonic_envelope'),
                ('/cscan_ni_status', '/cscan_ni_status'),
            ],
            output='screen',
        ),
    ])
