#!/usr/bin/env python3
"""
RViz + MuJoCo Joint Bridge Node

订阅 /joint_states 话题，将 GoFa CRB15000 机械臂在 MuJoCo 中可视化。

核心功能：
1. 订阅 /joint_states (来自 ros2_control 或 RViz)
2. 解析关节位置 (joint_1 到 joint_6)
3. 更新 MuJoCo 模型状态
4. 渲染并保存图像

使用方法:
    # 终端1: 启动 RViz + MoveIt
    ros2 launch abb_gofa_crb15000_10_152_moveit_config mujoco_demo.launch.py rviz:=true

    # 终端2: 运行 bridge 节点
    ros2 run quintic_trajectory_planner rviz_mujoco_bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import numpy as np
import os
import imageio


class RVizMuJoCoBridge(Node):
    def __init__(self):
        super().__init__("rviz_mujoco_bridge")

        # 参数
        self.declare_parameter("output_dir", "/home/i/ros2_ws")
        self.declare_parameter("render_interval", 3)  # 每隔几次回调渲染一次
        self.declare_parameter("verbose", True)

        self.output_dir = self.get_parameter("output_dir").value
        self.render_interval = self.get_parameter("render_interval").value
        self.verbose = self.get_parameter("verbose").value

        # 订阅 joint_states
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )

        self.get_logger().info("RViz-MuJoCo Bridge initialized")
        self.get_logger().info(f"Subscribing to /joint_states")

        # 状态
        self.frame_count = 0
        self.last_render_time = self.get_clock().now()

        # 创建 MuJoCo 模型
        self.create_mujoco_model()

        # 关节位置映射
        self.current_qpos = np.zeros(6)

        # 线程安全
        self._data_mutex = threading.Lock()

    def create_mujoco_model(self):
        """创建 GoFa CRB15000 MuJoCo 模型"""
        xml = """
        <mujoco model="gofa_crb15000">
            <compiler angle="radian" autolimits="true"/>
            <option timestep="0.001"/>

            <visual>
                <global offwidth="800" offheight="600"/>
                <quality shadow="true"/>
            </visual>

            <default>
                <geom friction="0.5" damping="0.1"/>
            </default>

            <worldbody>
                <!-- Ground -->
                <body name="ground" pos="0 0 -0.001">
                    <geom type="plane" size="2 2" rgba="0.9 0.9 0.9 1"/>
                </body>

                <!-- Base -->
                <body name="base_link" pos="0 0 0">
                    <geom type="cylinder" size="0.15 0.1" mass="5" rgba="0.3 0.3 0.3 1"/>

                    <!-- Joint 1 - Base Rotation (Z-axis) -->
                    <body name="link_1" pos="0 0 0.218">
                        <inertial pos="0 0 0" mass="11.8" diaginertia="0.1 0.1 0.1"/>
                        <geom type="cylinder" size="0.06 0.18" pos="0 0 -0.09" rgba="1 0.85 0 1"/>
                        <joint name="joint_1" type="hinge" axis="0 0 1" range="-4.712 4.712" damping="0"/>

                        <!-- Joint 2 - Shoulder Pitch (Y-axis) -->
                        <body name="link_2" pos="0 0.181 0">
                            <inertial pos="0 0.1 0.05" mass="17.5" diaginertia="0.5 0.5 0.1"/>
                            <geom type="box" size="0.07 0.18 0.12" pos="0 0.08 0.08" rgba="1 0.85 0 1"/>
                            <joint name="joint_2" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.268 2.268" damping="0"/>

                            <!-- Joint 3 - Elbow (Y-axis) -->
                            <body name="link_3" pos="0 0 0.260">
                                <inertial pos="0 0 0.1" mass="7.5" diaginertia="0.1 0.1 0.05"/>
                                <geom type="box" size="0.055 0.15 0.18" pos="0 0 0.09" rgba="1 0.85 0 1"/>
                                <joint name="joint_3" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.356 2.356" damping="0"/>

                                <!-- Joint 4 - Wrist 1 (Z-axis) -->
                                <body name="link_4" pos="0 0 0.200">
                                    <inertial pos="0 0 0.05" mass="2.7" diaginertia="0.02 0.02 0.01"/>
                                    <geom type="cylinder" size="0.04 0.1" pos="0 0 0.05" rgba="1 0.85 0 1"/>
                                    <joint name="joint_4" type="hinge" axis="0 0 1" pos="0 0 0" range="-3.490 3.490" damping="0"/>

                                    <!-- Joint 5 - Wrist 2 (Y-axis) -->
                                    <body name="link_5" pos="0 0.130 0">
                                        <inertial pos="0 0 0.03" mass="0.63" diaginertia="0.005 0.005 0.003"/>
                                        <geom type="box" size="0.035 0.08 0.08" pos="0 0 0.03" rgba="1 0.85 0 1"/>
                                        <joint name="joint_5" type="hinge" axis="0 1 0" pos="0 0 0" range="-2.356 2.356" damping="0"/>

                                        <!-- Joint 6 - Wrist 3 (Z-axis) -->
                                        <body name="link_6" pos="0 0.072 0">
                                            <inertial pos="0 0 0.02" mass="0.14" diaginertia="0.002 0.002 0.001"/>
                                            <geom type="cylinder" size="0.03 0.07" pos="0 0 0.02" rgba="1 0.85 0 1"/>
                                            <joint name="joint_6" type="hinge" axis="0 0 1" pos="0 0 0" range="-6.981 6.981" damping="0"/>

                                            <!-- Tool -->
                                            <body name="tool0" pos="0 0.060 0">
                                                <geom type="box" size="0.015 0.04 0.015" rgba="0 0 1 1"/>
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

        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)

        # 关节名称列表
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

        # 获取关节在 qpos 中的索引
        self.joint_qpos_ids = []
        for name in self.joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                # qpos 索引通过 jnt_qposadr 获取
                qpos_idx = self.model.jnt_qposadr[jid]
                self.joint_qpos_ids.append(qpos_idx)
            else:
                self.get_logger().warn(f"Joint {name} not found!")

        # 打印调试信息
        print(f"Model info: {self.model.nbody} bodies, {self.model.njnt} joints, {self.model.nq} qpos")
        print(f"Joint qpos indices: {self.joint_qpos_ids}")

        # 创建渲染器
        self.renderer = mujoco.Renderer(self.model, height=600, width=800)

        self.get_logger().info("MuJoCo model created successfully")

    def callback(self, msg: JointState):
        """处理 /joint_states 消息"""
        # 解析关节位置
        with self._data_mutex:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    self.current_qpos[idx] = msg.position[i]

            # 更新 MuJoCo 状态
            for i, qpos_id in enumerate(self.joint_qpos_ids):
                self.data.qpos[qpos_id] = self.current_qpos[i]

            # 前向运动学更新
            mujoco.mj_forward(self.model, self.data)

            # 定期渲染 (copy data for render outside lock)
            frame_count = self.frame_count + 1
            self.frame_count = frame_count

        # 渲染在锁外执行（耗时长）
        if frame_count % self.render_interval == 0:
            self.render()

        # 日志
        if self.verbose and frame_count % 50 == 0:
            with self._data_mutex:
                qpos_snapshot = list(self.current_qpos)
            self.get_logger().info(
                f"Qpos: {[f'{q:.3f}' for q in qpos_snapshot]}"
            )

    def render(self):
        """渲染当前状态"""
        self.renderer.update_scene(self.data)
        img = self.renderer.render()

        # 保存图像
        output_path = os.path.join(self.output_dir, "mujoco_bridge_view.png")
        try:
            imageio.imwrite(output_path, img)
        except Exception as e:
            self.get_logger().warn(f'Failed to write MuJoCo view: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RVizMuJoCoBridge()

    print("\n" + "=" * 60)
    print("RViz + MuJoCo Joint Bridge")
    print("=" * 60)
    print("\nInstructions:")
    print("1. Launch RViz with MoveIt:")
    print("   ros2 launch abb_gofa_crb15000_10_152_moveit_config mujoco_demo.launch.py")
    print("\n2. This node will subscribe to /joint_states and render in MuJoCo")
    print("\n3. Plan and execute motions in RViz to see the robot in MuJoCo")
    print("=" * 60 + "\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()