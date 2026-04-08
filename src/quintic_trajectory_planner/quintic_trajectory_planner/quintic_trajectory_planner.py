"""
Quintic Polynomial Trajectory Planner for ABB GoFa CRB15000

关节空间轨迹规划算法选择说明：
-----------------------------------------------
常用算法：
1. 梯形速度规划（Trapezoidal）- 计算简单但加速度不连续，有冲击
2. S曲线规划（S-curve）- 加速度连续但计算复杂
3. 五次多项式（Quintic Polynomial）- 位置、速度、加速度均连续，平滑性最好
4. 七次多项式 - jerk也连续，但计算量大

本节点选择 **五次多项式** 作为核心算法，原因：
- 边界条件：位置、速度、加速度六个约束刚好确定五次多项式系数
- 轨迹平滑：位置、速度、加速度曲线连续无突变
- 计算高效：只需解线性方程组，实时性好
- 适合机械臂：关节空间规划无需考虑奇异位姿和障碍物

Author: ROS2 Trajectory Planner
"""

import math
import numpy as np
from typing import List, Tuple

class QuinticTrajectoryPlanner:
    """
    五次多项式轨迹规划器

    对于每个关节，从初始状态 (q0, qd0, qdd0) 到目标状态 (q1, qd1, qdd1)
    构造五次多项式轨迹：
        q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

    通过边界条件求解系数，生成平滑的关节轨迹。
    """

    def __init__(self, num_joints: int = 6):
        self.num_joints = num_joints

    def compute_quintic_coefficients(
        self,
        q0: float, qd0: float, qdd0: float,
        q1: float, qd1: float, qdd1: float,
        T: float
    ) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]:
        """
        计算五次多项式系数

        边界条件：
            q(0)   = q0  (初始位置)
            q(0)'  = qd0 (初始速度)
            q(0)'' = qdd0(初始加速度)
            q(T)   = q1  (目标位置)
            q(T)'  = qd1 (目标速度)
            q(T)'' = qdd1(目标加速度)

        解线性方程组得到系数 a0...a5
        """
        if T <= 0:
            raise ValueError(f"Duration T must be positive, got {T}")

        # 构建系数矩阵 (6x6)
        # [t^0, t^1, t^2,  t^3,   t^4,    t^5]
        A = np.array([
            [1,  0,   0,    0,     0,      0],    # q(0) = q0
            [0,  1,   0,    0,     0,      0],    # q'(0) = qd0
            [0,  0,   2,    0,     0,      0],    # q''(0) = qdd0
            [1,  T,  T**2, T**3,  T**4,  T**5],   # q(T) = q1
            [0,  1,  2*T, 3*T**2, 4*T**3, 5*T**4], # q'(T) = qd1
            [0,  0,   2,  6*T,  12*T**2,20*T**3]  # q''(T) = qdd1
        ])

        # 右侧常数向量
        b = np.array([q0, qd0, qdd0, q1, qd1, qdd1])

        # 求解线性方程组
        try:
            coeffs = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            # 如果矩阵奇异，使用伪逆
            coeffs = np.linalg.lstsq(A, b, rcond=None)[0]

        a0, a1, a2, a3, a4, a5 = coeffs
        return [a0], [a1], [a2], [a3], [a4], [a5]

    def compute_trajectory(
        self,
        current_joints: List[float],
        target_joints: List[float],
        duration: float,
        num_points: int = 100,
        initial_velocities: List[float] = None,
        final_velocities: List[float] = None,
        initial_accelerations: List[float] = None,
        final_accelerations: List[float] = None
    ) -> Tuple[List[List[float]], List[List[float]], List[List[float]], List[float]]:
        """
        计算完整轨迹

        Args:
            current_joints: 当前关节角度 [rad]
            target_joints: 目标关节角度 [rad]
            duration: 轨迹时长 [s]
            num_points: 轨迹点数
            initial_velocities: 初始速度 (默认0)
            final_velocities: 终止速度 (默认0)
            initial_accelerations: 初始加速度 (默认0)
            final_accelerations: 终止加速度 (默认0)

        Returns:
            positions: 位置列表 [[j1, j2, ...], ...]
            velocities: 速度列表
            accelerations: 加速度列表
            time_stamps: 时间戳列表
        """
        if len(current_joints) != self.num_joints or len(target_joints) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joints, got {len(current_joints)} and {len(target_joints)}")

        # NaN/Inf validation
        for i, (curr, target) in enumerate(zip(current_joints, target_joints)):
            if not math.isfinite(curr) or not math.isfinite(target):
                raise ValueError(f"Invalid value at joint {i}: current={curr}, target={target}")

        # 默认边界条件：静止起停
        if initial_velocities is None:
            initial_velocities = [0.0] * self.num_joints
        if final_velocities is None:
            final_velocities = [0.0] * self.num_joints
        if initial_accelerations is None:
            initial_accelerations = [0.0] * self.num_joints
        if final_accelerations is None:
            final_accelerations = [0.0] * self.num_joints

        # 计算每个关节的五次多项式系数
        joint_coefficients = []
        for i in range(self.num_joints):
            coeffs = self.compute_quintic_coefficients(
                current_joints[i], initial_velocities[i], initial_accelerations[i],
                target_joints[i], final_velocities[i], final_accelerations[i],
                duration
            )
            joint_coefficients.append(coeffs)

        # 采样轨迹点
        positions = []
        velocities = []
        accelerations = []
        time_stamps = []

        # Guard against num_points <= 1 (division by zero)
        if num_points <= 1:
            return ([current_joints] * 1,
                    [[0.0] * self.num_joints] * 1,
                    [[0.0] * self.num_joints] * 1,
                    [0.0])

        for k in range(num_points):
            t = k * duration / (num_points - 1)
            time_stamps.append(t)

            joint_pos = []
            joint_vel = []
            joint_acc = []

            for i in range(self.num_joints):
                a0, a1, a2, a3, a4, a5 = joint_coefficients[i]

                # 位置 q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
                q = (a0[0] + a1[0]*t + a2[0]*t**2 + a3[0]*t**3 +
                     a4[0]*t**4 + a5[0]*t**5)

                # 速度 q'(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
                qd = (a1[0] + 2*a2[0]*t + 3*a3[0]*t**2 +
                      4*a4[0]*t**3 + 5*a5[0]*t**4)

                # 加速度 q''(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
                qdd = (2*a2[0] + 6*a3[0]*t + 12*a4[0]*t**2 +
                       20*a5[0]*t**3)

                joint_pos.append(q)
                joint_vel.append(qd)
                joint_acc.append(qdd)

            positions.append(joint_pos)
            velocities.append(joint_vel)
            accelerations.append(joint_acc)

        return positions, velocities, accelerations, time_stamps

    def compute_trajectory_with_velocity_limits(
        self,
        current_joints: List[float],
        target_joints: List[float],
        max_velocity: float = 2.0,
        max_acceleration: float = 5.0,
        num_points: int = 100
    ) -> Tuple[List[List[float]], List[List[float]], List[List[float]], List[float], float]:
        """
        带速度/加速度约束的五次多项式轨迹规划

        自动计算满足约束的最大时间

        Args:
            current_joints: 当前关节角度 [rad]
            target_joints: 目标关节角度 [rad]
            max_velocity: 最大允许速度 [rad/s]
            max_acceleration: 最大允许加速度 [rad/s^2]
            num_points: 轨迹点数

        Returns:
            positions, velocities, accelerations, time_stamps, actual_duration
        """
        # 计算最短时间（基于速度约束）
        total_distance = sum(abs(target_joints[i] - current_joints[i])
                            for i in range(self.num_joints))

        # 如果距离为0，直接返回
        if total_distance < 1e-6:
            return ([current_joints] * num_points,
                    [[0.0] * self.num_joints] * num_points,
                    [[0.0] * self.num_joints] * num_points,
                    [i * 0.01 for i in range(num_points)],
                    0.0)

        # 安全检查：防止除零
        if max_velocity <= 0 or max_acceleration <= 0:
            raise ValueError(f'Invalid velocity/acceleration limits: v={max_velocity}, a={max_acceleration}')

        # 速度约束下的最短时间
        T_velocity = total_distance / max_velocity

        # 加速度约束下的最短时间（物理正确的梯形profile）
        # 关键距离 d_critical = v_max^2 / a_max（加速到v_max再减速到0所需的最短距离）
        d_critical = max_velocity ** 2 / max_acceleration
        if total_distance <= d_critical:
            # 距离不足以达到最大速度：纯三角形profile
            T_acceleration = 2.0 * math.sqrt(total_distance / max_acceleration)
        else:
            # 距离足够达到最大速度：梯形profile（匀速段 + 加速段 + 减速段）
            T_acceleration = total_distance / max_velocity + max_velocity / max_acceleration

        # 取较大值，确保满足两个约束，并加 20% 安全系数
        T = max(T_velocity, T_acceleration, 0.1) * 1.2

        return (*self.compute_trajectory(current_joints, target_joints, T, num_points), T)