from launch import LaunchDescription, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression


def generate_launch_description():
    # 声明参数
    approach_height_m = LaunchConfiguration('approach_height_m', default='0.05')
    alignment_timeout_sec = LaunchConfiguration('alignment_timeout_sec', default='2.0')
    max_retry = LaunchConfiguration('max_retry', default='3')
    alignment_threshold_m = LaunchConfiguration('alignment_threshold_m', default='0.005')
    ik_group_name = LaunchConfiguration('ik_group_name', default='manipulator')

    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'approach_height_m',
            default_value=approach_height_m,
            description='探头到电池的距离 (m)'
        ),
        DeclareLaunchArgument(
            'alignment_timeout_sec',
            default_value=alignment_timeout_sec,
            description='IK超时时间 (s)'
        ),
        DeclareLaunchArgument(
            'max_retry',
            default_value=max_retry,
            description='IK失败重试次数'
        ),
        DeclareLaunchArgument(
            'alignment_threshold_m',
            default_value=alignment_threshold_m,
            description='对齐容差 (m)'
        ),
        DeclareLaunchArgument(
            'ik_group_name',
            default_value=ik_group_name,
            description='MoveIt IK group名称'
        ),

        # 节点
        Node(
            package='abb_vision',
            executable='battery_alignment_controller',
            name='battery_alignment_controller',
            parameters=[{
                'approach_height_m': PythonExpression(['float(', approach_height_m, ')']),
                'alignment_timeout_sec': PythonExpression(['float(', alignment_timeout_sec, ')']),
                'max_retry': PythonExpression(['int(', max_retry, ')']),
                'alignment_threshold_m': PythonExpression(['float(', alignment_threshold_m, ')']),
                'ik_group_name': ik_group_name,
            }],
            remappings=[
                ('/battery_poses', '/battery_poses'),
                ('/egm/robot_pose', '/egm/robot_pose'),
                ('/joint_states', '/joint_states'),
                ('/target_joint_states', '/target_joint_states'),
            ],
            output='screen',
        ),
    ])
