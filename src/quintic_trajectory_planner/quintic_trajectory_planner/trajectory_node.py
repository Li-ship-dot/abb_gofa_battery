"""
Quintic Trajectory Planner ROS2 Node

功能：
- 订阅目标关节角度话题 /target_joint_states
- 使用五次多项式生成平滑轨迹
- 发布到 /joint_trajectory (JointTrajectory)
- 发布轨迹执行进度到 /trajectory_execution_progress

使用方法：
    ros2 run quintic_trajectory_planner trajectory_node --ros-args -p num_joints:=6
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

from quintic_trajectory_planner.quintic_trajectory_planner import QuinticTrajectoryPlanner


class QuinticTrajectoryNode(Node):
    """
    五次多项式轨迹规划节点

    订阅:
        /target_joint_states: 目标关节角度 (JointState)
        /current_joint_states: 当前关节角度 (JointState)

    发布:
        /joint_trajectory: 平滑后的轨迹 (JointTrajectory)
        /trajectory_execution_progress: 执行进度 (Float64MultiArray)
    """

    def __init__(self, node_name='quintic_trajectory_planner'):
        super().__init__(node_name)

        # 参数 - 注意：num_joints 必须在 joint_names 之前声明
        self.declare_parameter('num_joints', 6)
        self.num_joints = self.get_parameter('num_joints').get_parameter_value().integer_value
        self.declare_parameter('joint_names', [f'joint_{i+1}' for i in range(self.num_joints)])
        self.declare_parameter('default_duration', 2.0)  # 默认轨迹时长 [s]
        self.declare_parameter('num_trajectory_points', 100)  # 轨迹点数
        self.declare_parameter('max_velocity', 2.0)  # rad/s (全局上限)
        self.declare_parameter('max_velocity_joints', [2.0, 2.0, 2.0, 2.0, 2.0, 2.0])  # per-joint上限
        self.declare_parameter('max_acceleration', 5.0)  # rad/s^2
        self.declare_parameter('use_velocity_limits', True)

        self.default_duration = self.get_parameter('default_duration').get_parameter_value().double_value
        self.num_points = self.get_parameter('num_trajectory_points').get_parameter_value().integer_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.use_velocity_limits = self.get_parameter('use_velocity_limits').get_parameter_value().bool_value

        # Per-joint velocity limits (必须与 ros2_control xacro 中的 limits 一致)
        # joint_1: ±2.094, joint_2: ±2.09, joint_3: ±2.18, joint_4/5/6: ±3.49
        per_joint_vel = self.get_parameter('max_velocity_joints').get_parameter_value().double_array_value
        if len(per_joint_vel) >= self.num_joints:
            self.max_velocity_joints = list(per_joint_vel[:self.num_joints])
        else:
            self.max_velocity_joints = [self.max_velocity] * self.num_joints
        # 有效全局上限取所有关节上限的最大值——轨迹时长由最严格关节决定
        self.effective_max_velocity = max(self.max_velocity_joints)

        # 轨迹规划器
        self.planner = QuinticTrajectoryPlanner(self.num_joints)

        # 线程安全锁
        self._state_mutex = threading.Lock()

        # 当前关节状态
        self.current_joints = [0.0] * self.num_joints
        self.target_joints = None
        self.has_target = False
        self.last_current_msg_time = None  # 用于 /current_joint_states 看门狗

        # 轨迹执行状态
        self.current_trajectory = None
        self.trajectory_start_time = None
        self.trajectory_duration = 0.0

        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅者
        self.target_sub = self.create_subscription(
            JointState,
            '/target_joint_states',
            self.target_callback,
            qos_profile
        )

        self.current_sub = self.create_subscription(
            JointState,
            '/current_joint_states',
            self.current_callback,
            qos_profile
        )

        # 发布者
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            qos_profile
        )

        self.progress_pub = self.create_publisher(
            Float64MultiArray,
            '/trajectory_execution_progress',
            qos_profile
        )

        # 轨迹执行定时器 (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # /current_joint_states 看门狗定时器 (2Hz) - 检测关节状态停滞
        self.staleness_timer = self.create_timer(0.5, self.staleness_check_callback)

        self.get_logger().info(f'Quintic Trajectory Planner Node initialized')
        self.get_logger().info(f'  num_joints: {self.num_joints}')
        self.get_logger().info(f'  default_duration: {self.default_duration}s')
        self.get_logger().info(f'  max_velocity: {self.max_velocity} rad/s (global cap)')
        self.get_logger().info(f'  max_velocity_joints: {self.max_velocity_joints} rad/s (per-joint)')
        self.get_logger().info(f'  effective_max_velocity: {self.effective_max_velocity} rad/s')
        self.get_logger().info(f'  max_acceleration: {self.max_acceleration} rad/s^2')

    def target_callback(self, msg: JointState):
        """处理目标关节角度"""
        if len(msg.position) < self.num_joints:
            self.get_logger().warn(
                f'Received {len(msg.position)} joints, expected {self.num_joints}'
            )
            return

        # NaN/Inf validation
        if not all(math.isfinite(p) for p in msg.position):
            self.get_logger().error(f'Invalid joint state: contains NaN/Inf')
            return

        with self._state_mutex:
            self.target_joints = list(msg.position[:self.num_joints])
            self.has_target = True
            current_joints_snapshot = list(self.current_joints)
            target_joints_snapshot = list(self.target_joints)
            self.get_logger().info(f'Received target: {[f"{j:.3f}" for j in self.target_joints]}')

        # 计算轨迹（不在持有锁的情况下执行）
        self._compute_and_publish(current_joints_snapshot, target_joints_snapshot)

    def current_callback(self, msg: JointState):
        """处理当前关节角度反馈"""
        with self._state_mutex:
            if len(msg.position) >= self.num_joints:
                self.current_joints = list(msg.position[:self.num_joints])
                self.last_current_msg_time = self.get_clock().now()

    def _compute_and_publish(self, current_joints, target_joints):
        """内部方法：计算轨迹并发布（不含锁，需在外部调用方保证线程安全）"""
        if self.use_velocity_limits:
            # 使用 effective_max_velocity（= min(max_velocity, min(per_joint_limits))）
            # 确保不超过任何关节的物理速度上限
            planning_velocity = min(self.max_velocity, min(self.max_velocity_joints))
            positions, velocities, accelerations, time_stamps, duration = \
                self.planner.compute_trajectory_with_velocity_limits(
                    current_joints,
                    target_joints,
                    planning_velocity,
                    self.max_acceleration,
                    self.num_points
                )
        else:
            positions, velocities, accelerations, time_stamps = \
                self.planner.compute_trajectory(
                    current_joints,
                    target_joints,
                    self.default_duration,
                    self.num_points
                )
            duration = self.default_duration

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        if len(joint_names) != self.num_joints:
            joint_names = [f'joint_{i+1}' for i in range(self.num_joints)]
        trajectory_msg.joint_names = joint_names

        for i in range(len(positions)):
            point = JointTrajectoryPoint()
            point.positions = positions[i]
            point.velocities = velocities[i]
            point.accelerations = accelerations[i]
            nanosec = int(round((time_stamps[i] % 1) * 1e9))
            sec = int(time_stamps[i]) + (1 if nanosec >= 1000000000 else 0)
            nanosec = nanosec % 1000000000
            dur = Duration(sec=sec, nanosec=nanosec)
            point.time_from_start = dur
            trajectory_msg.points.append(point)

        self.trajectory_pub.publish(trajectory_msg)

        with self._state_mutex:
            self.current_trajectory = positions
            self.trajectory_start_time = self.get_clock().now()
            self.trajectory_duration = duration

        self.get_logger().info(
            f'Published trajectory: {len(positions)} points, duration: {duration:.2f}s'
        )

    def compute_and_publish_trajectory(self):
        """计算并发布轨迹（外部接口，已废弃，请使用 _compute_and_publish）"""
        with self._state_mutex:
            if not self.has_target or self.target_joints is None:
                return
            current_joints = list(self.current_joints)
            target_joints = list(self.target_joints)
        self._compute_and_publish(current_joints, target_joints)

    def timer_callback(self):
        """轨迹执行进度发布"""
        with self._state_mutex:
            if self.current_trajectory is None or self.trajectory_start_time is None:
                return
            current_trajectory = self.current_trajectory
            # P0-SW-3 Fix: capture start time as int64 nanoseconds to avoid
            # rclpy.time.Time subtraction type issues across mutex boundaries
            trajectory_start_ns = self.trajectory_start_time.nanoseconds
            trajectory_duration = self.trajectory_duration

        elapsed = (self.get_clock().now().nanoseconds - trajectory_start_ns) / 1e9
        progress = min(elapsed / trajectory_duration, 1.0) if trajectory_duration > 0 else 0.0

        progress_msg = Float64MultiArray()
        progress_msg.data = [
            float(progress),  # 0-1 进度
            float(elapsed),   # 已用时间
            float(trajectory_duration - elapsed) if trajectory_duration > elapsed else 0.0  # 剩余时间
        ]
        progress_msg.layout = MultiArrayLayout()
        progress_msg.layout.dim = [MultiArrayDimension(label="progress", size=3, stride=3)]

        self.progress_pub.publish(progress_msg)

        if progress >= 1.0:
            with self._state_mutex:
                self.current_trajectory = None
                self.trajectory_start_time = None
                self.trajectory_duration = 0.0

    def staleness_check_callback(self):
        """检查 /current_joint_states 是否停滞（看门狗）"""
        with self._state_mutex:
            if self.last_current_msg_time is None:
                return  # 尚未收到任何消息
            last_msg_ns = self.last_current_msg_time.nanoseconds
            now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - last_msg_ns) / 1e9
        if elapsed > 1.0:
                # /current_joint_states 超过1秒未更新，警告并清 has_target
                if self.has_target:
                    self.get_logger().warn(
                        f'/current_joint_states is stale ({elapsed:.1f}s since last update). '
                        f'Clearing target to prevent trajectory from stale state.'
                    )
                    self.has_target = False


def main(args=None):
    rclpy.init(args=args)

    node = QuinticTrajectoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()