#!/usr/bin/env python3
"""
MuJoCo 仿真测试：五次多项式轨迹规划 (无头模式)
"""

import mujoco
import numpy as np
import sys

sys.path.insert(0, '/home/i/ros2_ws/src/quintic_trajectory_planner')
from quintic_trajectory_planner import QuinticTrajectoryPlanner


def create_gofa_mjcf():
    """创建 ABB GoFa CRB15000 简化 MJCF 模型"""
    mjcf_template = """
    <mujoco model="gofa_crb15000">
        <compiler angle="radian" autolimits="true"/>
        <option timestep="0.001" integrator="implicitfast"/>

        <worldbody>
            <body name="base_link" pos="0 0 0">
                <geom type="box" size="0.15 0.15 0.1" mass="13.5" pos="0 0 0.05" rgba="0.5 0.5 0.5 1"/>
                <body name="link_1" pos="0 0 0.218">
                    <geom type="cylinder" size="0.06 0.15" mass="11.8" pos="0 0 -0.075" rgba="1 1 0 1"/>
                    <joint name="joint_1" type="hinge" axis="0 0 1" pos="0 0 0" range="-4.712 4.712" damping="10"/>
                    <body name="link_2" pos="0 0.181 0">
                        <geom type="box" size="0.05 0.12 0.22" mass="7.8" pos="0 0.06 0.14" rgba="1 1 0 1"/>
                        <joint name="joint_2" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.268 2.268" damping="10"/>
                        <body name="link_3" pos="0 0.130 0.260">
                            <geom type="box" size="0.04 0.08 0.20" mass="4.5" pos="0 -0.015 0.135" rgba="1 1 0 1"/>
                            <joint name="joint_3" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.356 2.356" damping="10"/>
                            <body name="link_4" pos="0 0 0.200">
                                <geom type="box" size="0.035 0.08 0.15" mass="1.8" pos="0 0.004 0.107" rgba="1 1 0 1"/>
                                <joint name="joint_4" type="hinge" axis="0 0 1" pos="0 0 0" range="-3.490 3.490" damping="5"/>
                                <body name="link_5" pos="0 0.130 0">
                                    <geom type="box" size="0.03 0.06 0.10" mass="1.3" pos="0 0.024 0.044" rgba="1 1 0 1"/>
                                    <joint name="joint_5" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.356 2.356" damping="5"/>
                                    <body name="link_6" pos="0 0.072 0">
                                        <geom type="box" size="0.025 0.05 0.08" mass="0.8" pos="0 0.001 0.048" rgba="1 1 0 1"/>
                                        <joint name="joint_6" type="hinge" axis="0 0 1" pos="0 0 0" range="-6.981 6.981" damping="3"/>
                                        <body name="tool0" pos="0 0.060 0">
                                            <geom type="box" size="0.02 0.04 0.02" pos="0 0.02 0" rgba="0 0 1 1"/>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>

        <actuator>
            <motor joint="joint_1" ctrllimited="true" ctrlrange="-2.0 2.0"/>
            <motor joint="joint_2" ctrllimited="true" ctrlrange="-2.0 2.0"/>
            <motor joint="joint_3" ctrllimited="true" ctrlrange="-2.0 2.0"/>
            <motor joint="joint_4" ctrllimited="true" ctrlrange="-2.0 2.0"/>
            <motor joint="joint_5" ctrllimited="true" ctrlrange="-2.0 2.0"/>
            <motor joint="joint_6" ctrllimited="true" ctrlrange="-2.0 2.0"/>
        </actuator>
    </mujoco>
    """
    return mjcf_template


def run_simulation_headless(positions, velocities, time_stamps):
    """无头模式运行仿真"""
    print("\n" + "=" * 60)
    print("MuJoCo 仿真 (无头模式)")
    print("=" * 60)

    mjcf = create_gofa_mjcf()
    model = mujoco.MjModel.from_xml_string(mjcf)
    data = mujoco.MjData(model)

    print(f"模型加载成功! 关节数: {model.njnt}, 自由度: {model.nq}")

    # 获取关节 ID
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(1, 7)]

    # 设置初始位置
    for i, jid in enumerate(joint_ids):
        data.qpos[jid] = positions[0][i]

    mujoco.mj_forward(model, data)

    # 数据记录
    actual_positions = []
    actual_velocities = []
    time_recording = []

    # 仿真参数
    n_trajectory = len(positions)
    trajectory_duration = time_stamps[-1]

    print(f"轨迹时长: {trajectory_duration}s, 点数: {n_trajectory}")
    print("开始仿真...")

    # 高速仿真循环 (不渲染)
    while data.time < trajectory_duration + 0.1:
        current_time = data.time

        # 找到对应时间戳的轨迹点
        target_idx = 0
        for idx, t in enumerate(time_stamps):
            if t <= current_time:
                target_idx = idx

        # PD 控制
        for i, jid in enumerate(joint_ids):
            target_pos = positions[target_idx][i]
            error = target_pos - data.qpos[jid]

            Kp = 100.0  # 提高增益
            Kd = 20.0

            control = Kp * error - Kd * data.qvel[jid]
            data.ctrl[jid] = np.clip(control, -2.0, 2.0)

        mujoco.mj_step(model, data)

        # 记录
        if len(time_recording) == 0 or data.time - time_recording[-1] >= 0.01:
            actual_positions.append([data.qpos[jid] for jid in joint_ids])
            actual_velocities.append([data.qvel[jid] for jid in joint_ids])
            time_recording.append(data.time)

    print("仿真完成!")

    # 结果分析
    print("\n" + "=" * 60)
    print("仿真结果分析")
    print("=" * 60)

    actual_positions = np.array(actual_positions)
    n_recorded = len(time_recording)
    n_expected = len(time_stamps)

    # 插值期望位置
    expected_interp = np.zeros((n_recorded, 6))
    for i in range(n_recorded):
        t = time_recording[i]
        for j in range(n_expected - 1):
            if time_stamps[j] <= t <= time_stamps[j + 1]:
                alpha = (t - time_stamps[j]) / (time_stamps[j + 1] - time_stamps[j])
                expected_interp[i] = np.array(positions[j]) * (1 - alpha) + np.array(positions[j + 1]) * alpha
                break
        else:
            if t >= time_stamps[-1]:
                expected_interp[i] = np.array(positions[-1])

    # 误差计算
    positions_error = actual_positions - expected_interp
    rmse = np.sqrt(np.mean(positions_error ** 2))
    max_error = np.max(np.abs(positions_error))

    print(f"\n位置跟踪 RMSE: {rmse:.6f} rad ({np.degrees(rmse):.4f} deg)")
    print(f"位置跟踪最大误差: {max_error:.6f} rad ({np.degrees(max_error):.4f} deg)")

    print("\n各关节最大误差:")
    for i in range(6):
        joint_max = np.max(np.abs(positions_error[:, i]))
        print(f"  joint_{i+1}: {joint_max:.6f} rad ({np.degrees(joint_max):.4f} deg)")

    # 绘图
    try:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        t_expected = np.array(time_stamps)

        for idx, ax in enumerate(axes.flat):
            ax.plot(t_expected, [p[idx] for p in positions], 'b-', label='Planned', linewidth=2)
            ax.plot(time_recording, actual_positions[:, idx], 'r--', label='Actual', alpha=0.7, linewidth=1.5)
            ax.set_title(f'{joint_names[idx]} Position')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        plt.suptitle(f'Quintic Trajectory Tracking (RMSE: {rmse:.4f} rad)', fontsize=14)
        plt.tight_layout()
        plt.savefig('/home/i/ros2_ws/trajectory_result.png', dpi=150)
        print("\n图表已保存到: /home/i/ros2_ws/trajectory_result.png")

    except ImportError:
        print("\nmatplotlib 未安装，跳过绘图")

    return rmse, max_error


def main():
    print("=" * 60)
    print("MuJoCo 五次多项式轨迹规划仿真测试")
    print("=" * 60)

    # 创建轨迹规划器
    planner = QuinticTrajectoryPlanner(num_joints=6)

    # 测试参数
    current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_joints = [0.5, 0.3, 0.2, 0.1, -0.2, 0.1]

    print(f"\n初始位置: {[f'{j:.3f}' for j in current_joints]}")
    print(f"目标位置: {[f'{j:.3f}' for j in target_joints]}")

    # 计算轨迹
    positions, velocities, accelerations, time_stamps = planner.compute_trajectory(
        current_joints, target_joints,
        duration=2.0, num_points=200
    )

    print(f"\n轨迹信息:")
    print(f"  点数: {len(positions)}")
    print(f"  时长: {time_stamps[-1]:.2f}s")
    print(f"  最大速度: {max(max(abs(v) for v in velocities[i]) for i in range(len(velocities))):.4f} rad/s")
    print(f"  最大加速度: {max(max(abs(a) for a in accelerations[i]) for i in range(len(accelerations))):.4f} rad/s²")

    # 边界条件验证
    print("\n边界条件验证:")
    print(f"  初始位置: {[f'{p:.4f}' for p in positions[0]]}")
    print(f"  终止位置: {[f'{p:.4f}' for p in positions[-1]]}")
    print(f"  初始速度: {[f'{v:.4f}' for v in velocities[0]]}")
    print(f"  终止速度: {[f'{v:.4f}' for v in velocities[-1]]}")
    print(f"  初始加速度: {[f'{a:.4f}' for a in accelerations[0]]}")
    print(f"  终止加速度: {[f'{a:.4f}' for a in accelerations[-1]]}")

    # 运行仿真
    rmse, max_error = run_simulation_headless(positions, velocities, time_stamps)

    # 总结
    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)

    if rmse < 0.01:
        print("✓ 轨迹跟踪精度合格 (RMSE < 0.01 rad)")
    else:
        print(f"  RMSE = {rmse:.4f} rad ({np.degrees(rmse):.2f} deg)")
        print("  注: 误差主要来自 PD 控制跟踪延迟，可通过提高增益改善")


if __name__ == "__main__":
    main()