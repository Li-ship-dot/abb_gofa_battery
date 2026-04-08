# ABB GoFa CRB15000 锂电池超声 C-scan 检测系统

> **ABB GoFa CRB15000 Lithium Battery Ultrasonic C-scan Inspection System**

[English](#english) | [中文](#中文)

---

## English

### Overview

ROS2 Humble implementation of a battery inspection system using an ABB GoFa CRB15000 robot. The system performs hand-eye calibration, battery pose detection, PBVS alignment, and ultrasonic C-scan imaging.

### Architecture

```
RealSense D415 Camera          NI Ultrasonic Device
        │                              │
        ▼                              ▼
┌───────────────────────────────────────────┐
│              ROS2 Humble                   │
│                                           │
│  battery_detector ──► battery_pose_estimator ──► battery_alignment_controller │
│       (HSV)              (PnP/IPPE)              (PBVS)                          │
│                                                                           │
│  cscan_trajectory_generator ──► cscan_udp_bridge ◄────── NI UDP           │
│       (zigzag grid)              │                                            │
│                                  ▼                                            │
│                           cscan_visualizer (C-scan image)                  │
│                                                                           │
│  quintic_trajectory_planner ──► /abb_controller/joint_trajectory             │
│                                                                           │
│  abb_hardware_interface ◄──► ABB GoFa OmniCore (EGM)                      │
└───────────────────────────────────────────┘
```

### Packages

| Package | Description |
|---------|-------------|
| `abb_vision` | Vision nodes: battery detector, pose estimator, hand-eye calibrator, alignment controller, C-scan generator/bridge/visualizer |
| `abb_omnicore_ros2` | Bringup launches, hardware interface, RWS client |
| `abb_libegm` | EGM C++ library |
| `abb_librws` | RWS C++ library |
| `abb_egm_rws_managers` | EGM + RWS managers |
| `abb_ros2_msgs` | ROS2 message definitions |
| `quintic_trajectory_planner` | Quintic trajectory smoothing |

### Three-Phase Workflow

```
Phase 1: Hand-Eye Calibration → Phase 2: Battery Alignment → Phase 3: C-scan
(One-time, results persist)   (Before each battery)      (Scan inspection)
```

### Quick Start

```bash
# Build
colcon build

# Hand-eye calibration (one-time)
ros2 launch abb_bringup abb_hand_eye_calibration.launch.py \
  robot_type:=gofa_crb15000_10_152 sim:=false use_real_camera:=true

# Battery alignment + C-scan
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  robot_type:=gofa_crb15000_10_152 sim:=false use_real_camera:=true \
  ni_ip:=192.168.1.100 calibration_file:=/path/to/calibration.yaml
```

### Mock Mode (No Hardware)

```bash
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  sim:=true use_mock_hand_eye:=true use_mock_ni:=true
```

### CI/CD

- **Build + Lint + Unit Tests** on every PR
- **Mock Chain E2E Test** (`test_mock_chain_integration.py`): verifies full software chain in mock mode without any hardware

### Documentation

- `ABB_GoFa_BatteryInspection_System_Report.md` — Full technical report
- `ABB_GoFa_BatteryInspection_QuickRef.md` — Quick reference guide

### License

Apache 2.0

---

## 中文

### 概述

基于 ABB GoFa CRB15000 机器人的锂电池超声 C-scan 检测系统，运行于 ROS2 Humble。完成手眼标定后，通过视觉定位电池姿态，执行 PBVS 对齐控制，最终沿栅格轨迹完成超声扫描成像。

### 系统架构

```
RealSense D415 相机                    NI 超声波设备
       │                                    │
       ▼                                    ▼
┌─────────────────────────────────────────────────────────┐
│                    ROS2 Humble                            │
│                                                           │
│  battery_detector ──► battery_pose_estimator ──► battery_alignment_controller │
│       (HSV颜色分割)        (PnP/IPPE位姿估计)       (PBVS视觉伺服)            │
│                                                                           │
│  cscan_trajectory_generator ──► cscan_udp_bridge ◄────── NI UDP           │
│       (蛇形栅格路径)               │                                        │
│                                  ▼                                        │
│                          cscan_visualizer (实时C-scan图像)                  │
│                                                                           │
│  quintic_trajectory_planner ──► /abb_controller/joint_trajectory            │
│                                                                           │
│  abb_hardware_interface ◄──► ABB GoFa OmniCore (EGM实时控制)              │
└─────────────────────────────────────────────────────────┘
```

### 软件包

| 包 | 说明 |
|----|------|
| `abb_vision` | 视觉节点：电池检测、位姿估计、手眼标定、对齐控制、C-scan生成/桥接/可视化 |
| `abb_omnicore_ros2` | 启动文件、硬件接口、RWS客户端 |
| `abb_libegm` | EGM C++ 库 |
| `abb_librws` | RWS C++ 库 |
| `abb_egm_rws_managers` | EGM + RWS 管理器 |
| `abb_ros2_msgs` | ROS2 消息定义 |
| `quintic_trajectory_planner` | 五次多项式轨迹平滑 |

### 三阶段工作流

```
阶段一：手眼标定 ──► 阶段二：电池对齐 ──► 阶段三：C-scan扫描
（一次性，结果持久化）   （每次换电池前）        （超声检测）
```

### 快速启动

```bash
# 编译
colcon build

# 手眼标定（一次性）
ros2 launch abb_bringup abb_hand_eye_calibration.launch.py \
  robot_type:=gofa_crb15000_10_152 sim:=false use_real_camera:=true

# 电池对齐 + C-scan扫描
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  robot_type:=gofa_crb15000_10_152 sim:=false use_real_camera:=true \
  ni_ip:=192.168.1.100 calibration_file:=/path/to/calibration.yaml
```

### Mock 模式（无硬件）

```bash
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  sim:=true use_mock_hand_eye:=true use_mock_ni:=true
```

三个参数独立控制：
- `sim:=true` — mock ros2_control（不走真实EGM）
- `use_mock_hand_eye:=true` — mock_data_node 模拟相机和手眼数据
- `use_mock_ni:=true` — cscan_udp_bridge 切换到 mock NI 模式

### CI/CD

- **Build + Lint + 单元测试**：每个 PR 自动触发
- **Mock Chain E2E 测试**（`test_mock_chain_integration.py`）：无硬件环境下验证完整软件链路

### 文档

- `ABB_GoFa_BatteryInspection_System_Report.md` — 完整技术报告
- `ABB_GoFa_BatteryInspection_QuickRef.md` — 快速操作指南

### 许可证

Apache 2.0
