#!/usr/bin/env python3
"""
MuJoCo + ROS2 Integration - 无头仿真模式

由于 Wayland 显示问题，使用无头模式完成仿真
"""

import mujoco
import numpy as np
import sys
sys.path.insert(0, '/home/i/ros2_ws/src/quintic_trajectory_planner')
from quintic_trajectory_planner import QuinticTrajectoryPlanner


def create_mujoco_robot():
    """Create MuJoCo robot model"""
    mjcf = """
    <mujoco model="gofa_visualization">
        <compiler angle="radian" autolimits="true"/>
        <option timestep="0.001" integrator="implicitfast"/>

        <worldbody>
            <body name="base_link" pos="0 0 0">
                <geom type="box" size="0.15 0.15 0.1" mass="13.5" pos="0 0 0.05" rgba="0.3 0.3 0.3 1"/>

                <body name="link_1" pos="0 0 0.218">
                    <geom type="cylinder" size="0.06 0.15" mass="11.8" pos="0 0 -0.075" rgba="1 0.8 0 1"/>
                    <joint name="joint_1" type="hinge" axis="0 0 1" pos="0 0 0" range="-4.712 4.712"/>

                    <body name="link_2" pos="0 0.181 0">
                        <geom type="box" size="0.05 0.12 0.22" mass="7.8" pos="0 0.06 0.14" rgba="1 0.8 0 1"/>
                        <joint name="joint_2" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.268 2.268"/>

                        <body name="link_3" pos="0 0.130 0.260">
                            <geom type="box" size="0.04 0.08 0.20" mass="4.5" pos="0 -0.015 0.135" rgba="1 0.8 0 1"/>
                            <joint name="joint_3" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.356 2.356"/>

                            <body name="link_4" pos="0 0 0.200">
                                <geom type="box" size="0.035 0.08 0.15" mass="1.8" pos="0 0.004 0.107" rgba="1 0.8 0 1"/>
                                <joint name="joint_4" type="hinge" axis="0 0 1" pos="0 0 0" range="-3.490 3.490"/>

                                <body name="link_5" pos="0 0.130 0">
                                    <geom type="box" size="0.03 0.06 0.10" mass="1.3" pos="0 0.024 0.044" rgba="1 0.8 0 1"/>
                                    <joint name="joint_5" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.356 2.356"/>

                                    <body name="link_6" pos="0 0.072 0">
                                        <geom type="box" size="0.025 0.05 0.08" mass="0.8" pos="0 0.001 0.048" rgba="1 0.8 0 1"/>
                                        <joint name="joint_6" type="hinge" axis="0 0 1" pos="0 0 0" range="-6.981 6.981"/>

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
    </mujoco>
    """
    return mjcf


def main():
    print("=" * 60)
    print("MuJoCo + Quintic Trajectory - Headless Mode")
    print("=" * 60)

    model = mujoco.MjModel.from_xml_string(create_mujoco_robot())
    data = mujoco.MjData(model)

    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")
        for i in range(1, 7)
    ]

    print(f"Model: {model.njnt} joints, {model.nq} DOF")

    # 轨迹规划
    planner = QuinticTrajectoryPlanner(num_joints=6)
    current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_joints = [0.5, 0.3, 0.2, 0.1, -0.2, 0.1]

    positions, velocities, accelerations, time_stamps = planner.compute_trajectory(
        current_joints, target_joints,
        duration=2.0,
        num_points=200
    )

    print(f"\nTrajectory: {len(positions)} points, {time_stamps[-1]:.1f}s")
    print(f"Max velocity: {max(max(abs(v) for v in velocities[i]) for i in range(len(velocities))):.3f} rad/s")
    print(f"Max acceleration: {max(max(abs(a) for a in accelerations[i]) for i in range(len(accelerations))):.3f} rad/s²")

    # 仿真循环
    recorded_positions = []
    recorded_velocities = []
    recorded_time = []
    target_positions = []

    trajectory_duration = time_stamps[-1]
    sim_start_time = data.time

    print("\nRunning simulation...")

    step = 0
    while data.time - sim_start_time < trajectory_duration + 0.1:
        current_sim_time = data.time - sim_start_time

        # 找对应轨迹点
        target_idx = 0
        for idx, t in enumerate(time_stamps):
            if t <= current_sim_time:
                target_idx = idx

        # 直接位置控制
        for i, jid in enumerate(joint_ids):
            if target_idx < len(positions):
                data.qpos[jid] = positions[target_idx][i]
            else:
                data.qpos[jid] = positions[-1][i]

        mujoco.mj_step(model, data)
        step += 1

        # 记录
        if len(recorded_time) == 0 or data.time - recorded_time[-1] >= 0.005:
            recorded_positions.append([data.qpos[jid] for jid in joint_ids])
            recorded_velocities.append([data.qvel[jid] for jid in joint_ids])
            recorded_time.append(current_sim_time)
            if target_idx < len(positions):
                target_positions.append(positions[target_idx])
            else:
                target_positions.append(positions[-1])

        if step % 500 == 0:
            progress = min(current_sim_time / trajectory_duration * 100, 100)
            print(f"Progress: {progress:.1f}% | Time: {current_sim_time:.2f}s")

    print(f"\nSimulation completed in {step} steps!")

    # 分析
    recorded = np.array(recorded_positions)
    targets = np.array(target_positions)
    times = np.array(recorded_time)
    min_len = min(len(recorded), len(targets))

    errors = recorded[:min_len] - targets[:min_len]
    rmse = np.sqrt(np.mean(errors ** 2))
    max_error = np.max(np.abs(errors))

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"\nTracking Performance:")
    print(f"  RMSE: {rmse:.6f} rad ({np.degrees(rmse):.4f} deg)")
    print(f"  Max Error: {max_error:.6f} rad ({np.degrees(max_error):.4f} deg)")

    print(f"\nJoint Performance:")
    for i in range(6):
        joint_rmse = np.sqrt(np.mean(errors[:, i] ** 2))
        print(f"  joint_{i+1}: RMSE={joint_rmse:.6f} rad")

    # 绘图
    try:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
        t_traj = np.array(time_stamps)

        for idx, ax in enumerate(axes.flat):
            ax.plot(t_traj, [p[idx] for p in positions], 'b-', linewidth=2, label='Planned')
            ax.plot(times[:min_len], recorded[:min_len, idx], 'r--', linewidth=1.5, alpha=0.8, label='Executed')
            ax.set_title(f'{joint_names[idx]} Position Tracking', fontsize=11)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        plt.suptitle(f'Quintic Trajectory in MuJoCo (RMSE: {rmse:.6f} rad)', fontsize=14)
        plt.tight_layout()
        plt.savefig('/home/i/ros2_ws/mujoco_ros2_integration.png', dpi=150)
        print(f"\nSaved: /home/i/ros2_ws/mujoco_ros2_integration.png")

        fig2, axes2 = plt.subplots(2, 1, figsize=(12, 8))

        ax = axes2[0]
        for i in range(6):
            ax.plot(times[:min_len], np.array(recorded_velocities)[:min_len, i], label=f'J{i+1}', linewidth=1.5)
        ax.set_title('Joint Velocities')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes2[1]
        error_norms = np.linalg.norm(errors, axis=1) * 1000  # mrad
        ax.plot(times[:min_len], error_norms, 'g-', linewidth=2, label='Error (mrad)')
        ax.axhline(y=rmse*1000, color='r', linestyle='--', label=f'RMSE: {rmse*1000:.2f} mrad')
        ax.set_title('Position Tracking Error')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (mrad)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.suptitle('Trajectory Execution Analysis', fontsize=14)
        plt.tight_layout()
        plt.savefig('/home/i/ros2_ws/mujoco_trajectory_analysis.png', dpi=150)
        print(f"Saved: /home/i/ros2_ws/mujoco_trajectory_analysis.png")

    except ImportError:
        print("\nmatplotlib not available")

    print("\n" + "=" * 60)
    print("DONE")
    print("=" * 60)


if __name__ == "__main__":
    main()