# ABB GoFa CRB15000 锂电池超声检测系统
## 技术报告与操作指南

**项目代号**：Battery Inspection ROS2 System
**版本**：v1.10 (Sprint 3 Review Round 2 Complete)
**日期**：2026-04-08
**更新内容**：cv::FileStorage cv::Exception未捕获导致battery节点崩溃(P0×3修复)；abb_alignment_bringup battery_detector/pose_estimator节点条件缺失；abb_cscan_bringup battery_detector mock模式冲突；全mock链路(sim+use_mock_hand_eye+use_mock_ni)验证通过
**硬件平台**：ABB GoFa CRB15000 OmniCore + Intel RealSense **D415** (global shutter)
**软件平台**：ROS2 Humble + MoveIt2

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件架构](#2-硬件架构)
3. [软件架构](#3-软件架构)
4. [数据流与通信拓扑](#4-数据流与通信拓扑)
5. [节点详解](#5-节点详解)
6. [NI UDP 通信协议](#6-ni-udp-通信协议)
7. [启动流程](#7-启动流程)
8. [操作指南](#8-操作指南)
9. [参数配置](#9-参数配置)
10. [三阶段工作流](#10-三阶段工作流)
11. [故障排查](#11-故障排查)
12. [技术约束与已知问题](#12-技术约束与已知问题)

---

## 1. 系统概述

### 1.1 项目目标

实现基于 ABB GoFa CRB15000 机器人的锂电池超声非接触式 C-scan 检测系统。系统完成手眼标定后，通过视觉定位电池姿态，执行 PBVS 对齐，最终沿栅格轨迹完成超声扫描。

### 1.2 核心功能

| 功能 | 描述 |
|------|------|
| 手眼标定 | ArUco 标记 + Tsai 算法，建立相机与机器人基座的坐标变换 |
| 电池检测 | HSV 颜色分割 + 轮廓分析，输出电池 4 角点像素坐标 |
| 3D 位姿估计 | PnP 算法，将电池角点从像素坐标映射到机器人 base_link 坐标系 |
| PBVS 对齐控制 | 视觉伺服控制，将机器人 TCP 移动至电池正上方 |
| 栅格轨迹生成 | 蛇形 zigzag 路径，覆盖整个电池表面 |
| NI UDP 桥接 | ROS2 与 NI 硬件之间的 UDP 数据通路 |
| 实时 C-scan 可视化 | 构建 C-scan 图像，实时显示扫描进度 |

### 1.3 系统三阶段

```
阶段一：手眼标定    →    阶段二：电池对齐    →    阶段三：C-scan 扫描
(Hand-Eye Calibration)      (Alignment)              (C-scan)
```

---

## 2. 硬件架构

### 2.1 系统连接图

```
┌──────────────────┐          ┌──────────────────┐
│   ABB GoFa       │          │   Intel          │
│   CRB15000       │          │   RealSense D455 │
│   OmniCore       │          │   (相机)          │
│                  │          └────────┬─────────┘
│  EGM (UDP)       │                   │ USB3
│  192.168.125.1   │                   │
└────────┬─────────┘                   │
         │ joint_states           /camera/image_raw
         │ /target_joint_states         │
         │ EGM 实时控制                 ▼
         │                    ┌──────────────────┐
         │                    │   ROS2 主机      │
         │                    │   (ubuntu 22.04) │
         │                    └────────┬─────────┘
         │                             │ UDP
         │                    ┌────────▼─────────┐
         │                    │   NI 示波器/主机  │
         │                    │   192.168.1.100  │
         │                    └──────────────────┘
         │                             │
         │                      超声脉冲/回波
         │                             │
         │                    ┌────────▼─────────┐
         │                    │  TecLab 空气耦合  │
         │                    │  超声探头         │
         └────────────────────┴──────────────────┘
```

### 2.2 网络配置

| 设备 | IP 地址 | 用途 |
|------|---------|------|
| ABB OmniCore | 192.168.125.1 | EGM 实时运动控制 |
| ROS2 主机 | 192.168.125.100 | 网桥/所有 ROS2 节点 |
| NI 硬件 | 192.168.1.100 | 超声数据采集 |
| RealSense D415 | — | USB3 连接，不占 IP |

### 2.3 关键硬件参数

| 参数 | 值 |
|------|-----|
| 机器人型号 | ABB GoFa CRB15000-10/1.52 |
| 相机型号 | Intel RealSense D415 |
| 相机深度精度 | ±0.5mm @ 400mm (D415深度精度优于D455) |
| 电池尺寸 | 70mm × 25mm（默认值） |
| 超声探头 | TecLab 空气耦合探头 |
| 超声收发 | Ritec（独立运行） |
| NI 硬件 | 示波器/主机（UDP 通信） |

---

## 3. 软件架构

### 3.1 ROS2 包结构

```
/home/i/ros2_ws/src/
├── abb_vision/                    # 视觉系统（7个节点）
│   ├── src/
│   │   ├── battery_detector.cpp
│   │   ├── battery_pose_estimator.cpp
│   │   ├── hand_eye_calibrator.cpp
│   │   ├── battery_alignment_controller.cpp
│   │   ├── cscan_trajectory_generator.cpp
│   │   ├── cscan_udp_bridge.cpp
│   │   └── cscan_visualizer.cpp
│   ├── msg/
│   │   └── CscanStatus.msg
│   ├── launch/
│   │   └── [7个独立launch文件]
│   └── config/
│       └── d415_camera_info.yaml.template
│
├── abb_omnicore_ros2/             # 机器人接口
│   ├── abb_bringup/
│   │   ├── launch/
│   │   │   ├── abb_hand_eye_calibration.launch.py
│   │   │   ├── abb_alignment_bringup.launch.py
│   │   │   └── abb_cscan_bringup.launch.py
│   │   └── test/                  # Launch smoke tests (2026-04-07 新增)
│   │       ├── test_abb_cscan_bringup_launch.py
│   │       ├── test_abb_alignment_bringup_launch.py
│   │       └── test_abb_hand_eye_calibration_launch.py
│   └── abb_hardware_interface/   # ros2_control hardware interface
│
├── quintic_trajectory_planner/    # 五次多项式轨迹规划
│   └── quintic_trajectory_planner.py
│
├── abb_libegm/                    # EGM C++库
├── abb_librws/                    # RWS C++库
├── abb_ros2_msgs/                 # 消息定义
└── abb_egm_rws_managers/          # EGM+RWS 管理器

# CI/CD 配置 (2026-04-07 新增)
.github/
└── workflows/
    └── ci.yml                    # GitHub Actions CI (ROS2 Humble)

```

### 3.2 abb_vision 节点总览

| # | 节点名 | 可执行文件 | 核心功能 |
|---|--------|-----------|---------|
| 1 | battery_detector | battery_detector | HSV 颜色分割检测电池 4 角点 |
| 2 | battery_pose_estimator | battery_pose_estimator | PnP 3D 位姿估计 |
| 3 | hand_eye_calibrator | hand_eye_calibrator | ArUco 手眼标定 |
| 4 | battery_alignment_controller | battery_alignment_controller | PBVS 视觉伺服对齐 |
| 5 | cscan_trajectory_generator | cscan_trajectory_generator | 栅格扫描轨迹生成 |
| 6 | cscan_udp_bridge | cscan_udp_bridge | NI ↔ ROS2 UDP 桥接 |
| 7 | cscan_visualizer | cscan_visualizer | C-scan 实时可视化 |

---

## 4. 数据流与通信拓扑

### 4.1 完整数据流图

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        RealSense D455                                       │
│                           /camera/image_raw                                  │
└────────────────────────────────┬─────────────────────────────────────────────┘
                                 │
     ┌────────────────────────────┼────────────────────────────┐
     │                            │                            │
     ▼                            ▼                            │
┌─────────────┐         ┌─────────────────┐          ┌─────────────────┐
│  hand_eye   │         │    battery      │          │     ...         │
│  calibrator │         │    detector     │          │                 │
│             │         │                 │          │                 │
│ /egm/robot  │         │ /battery_bboxes │          │                 │
│   _pose     │         │ /battery_detect │          │                 │
│ (订阅)       │         │   ions          │          │                 │
└──────┬──────┘         └────────┬────────┘          └─────────────────┘
       │ TF: base_link            │
       │ → camera_optical         │
       │   _frame                 │
       │                          │
       │              ┌───────────┴───────────┐
       │              │                       │
       ▼              ▼                       │
┌─────────────────────────┐                   │
│  battery_pose_estimator │◄──────────────────┘
│                         │
│ /battery_poses          │  /joint_states
│ (发布)                   │  (订阅)
└──────┬──────────────────┘        │
       │                            │
       │              ┌─────────────┴─────────────┐
       │              │                           │
       ▼              ▼                           │
┌────────────────────────────┐                    │
│  battery_alignment         │                    │
│  controller                │                    │
│                            │                    │
│  /target_joint_states ────┼────────────────────┤
│  (发布)                    │                    │
└────────────┬───────────────┘                    │
             │                                    │
             │        ┌───────────────────────────┘
             ▼        ▼
┌─────────────────────────────────────────────┐
│       quintic_trajectory_planner              │
│       (Python, trajectory smoothing)         │
│                                             │
│       /abb_controller/joint_trajectory        │
└────────────────────┬────────────────────────┘
                     │ EGM/UDP
                     ▼
┌─────────────────────────────────────────────┐
│          ABB GoFa OmniCore                  │
│          /joint_states (反馈)               │
└─────────────────────────────────────────────┘

  [对齐完成后，cscan 链路启动]

┌─────────────────────────────┐
│  cscan_trajectory_generator │◄── /battery_poses
│                             │
│  /cscan_grid_trigger (Int32)│
│  /cscan_status              │
└──────┬──────────────────────┘
       │
       ├──────────────────────────┐
       │                          │
       ▼                          ▼
┌────────────────┐       ┌────────────────────────┐
│ cscan_udp_     │       │   cscan_visualizer     │
│ bridge         │       │                        │
│                │       │   /cscan_live_image    │
│ UDP → NI:5001 │       │   /ascan_display       │
│                │       └────────────────────────┘
│ NI:5000 → UDP │
└───────┬────────┘
        │
        ▼
┌─────────────────────────────────────────────┐
│              NI 硬件                         │
│  超声采集 → A-scan 数据包 (UDP)              │
└─────────────────────────────────────────────┘
```

### 4.2 Topic 匹配表

| 发布节点 | Topic | 消息类型 | 订阅节点 | 状态 |
|---------|-------|---------|---------|------|
| battery_detector | /battery_bboxes | PoseArray | battery_pose_estimator | ✅ |
| battery_detector | /battery_detections | PoseArray | — | ✅ |
| battery_pose_estimator | /battery_poses | PoseArray | battery_alignment_controller, cscan_trajectory_generator | ✅ |
| hand_eye_calibrator | TF: base_link→camera | TransformStamped | battery_pose_estimator (via TF2) | ✅ |
| joint_to_cartesian_pose_node | /egm/robot_pose | Pose | hand_eye_calibrator, battery_alignment_controller | ✅ |
| battery_alignment_controller | /target_joint_states | JointState | quintic_trajectory_planner | ✅ |
| quintic_trajectory_planner | /abb_controller/joint_trajectory | Trajectory | abb_controller (JointTrajectoryController) | ✅ |
| battery_alignment_controller | /alignment_status | Bool | — | ✅ |
| cscan_trajectory_generator | /cscan_grid_trigger | Int32 | cscan_udp_bridge, cscan_visualizer | ✅ |
| cscan_trajectory_generator | /cscan_status | CscanStatus | cscan_visualizer | ✅ |
| cscan_udp_bridge | /ultrasonic_data | Float32MultiArray | cscan_visualizer | ✅ |
| cscan_udp_bridge | /ultrasonic_envelope | Float32 | — | ✅ |
| cscan_visualizer | /cscan_live_image | Image | — | ✅ |
| cscan_visualizer | /ascan_display | Image | — | ✅ |

---

## 5. 节点详解

### 5.1 battery_detector

**功能**：从相机图像中检测锂电池，输出 4 角点像素坐标。

**算法流程**：
1. BGR → HSV 色彩空间转换
2. 橙色掩码（HSV: H=0-40, S=100-255, V=100-255；含自适应宽化机制，上限 H=0-50）
3. Canny 边缘检测
4. 轮廓提取与形状过滤（面积 5000-100000，周长>100，长宽比≈2.8）
5. RotatedRect 角点输出

**订阅**：`/camera/image_raw` (Image)

**发布**：
- `/battery_bboxes` (PoseArray) — 4 角点坐标
- `/battery_detections` (PoseArray) — 电池中心 + 朝向角

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| debug_mode | true | 调试图像输出 |
| min_area | 5000.0 | 最小轮廓面积（像素） |
| max_area | 100000.0 | 最大轮廓面积（像素） |
| battery_length_mm | 70.0 | 电池长度 |
| battery_width_mm | 25.0 | 电池宽度 |
| hsv_h_min | **0** | HSV H 最小值（2026-04-07 更新，原5） |
| hsv_h_max | **40** | HSV H 最大值（2026-04-07 更新，原25） |
| hsv_s_min | 100 | HSV S 最小值 |
| hsv_s_max | 255 | HSV S 最大值 |
| hsv_v_min | 100 | HSV V 最小值 |
| hsv_v_max | 255 | HSV V 最大值 |

> **HSV 参数已外部化**：通过 `abb_alignment_bringup.launch.py` 和 `abb_cscan_bringup.launch.py` 的 launch 参数直接配置，无需修改代码。
> **HSV 自适应机制（2026-04-07 新增）**：battery_detector 内置光照自适应，连续30帧失败自动宽化H范围(上限[0,50])，连续60帧成功逐步收紧。支持运行时动态调参（`ros2 param set /battery_detector hsv_h_min 10`），诊断topic `/battery_detector/hsv_adaptive_status` 发布实时状态。

---

### 5.2 hand_eye_calibrator

**功能**：通过 ArUco 标记执行 Tsai 手眼标定，建立 base_link → camera_optical_frame 的坐标变换。

**标定流程**：
1. 机械臂移动至不同位姿（建议移动>30cm，旋转>15°）
2. 同步采集：相机图像（ArUco角点）+ 机械臂TCP位姿（/egm/robot_pose）
3. 采集 ≥10 组样本后调用 `/calibrate` 服务
4. Tsai 算法计算坐标变换（旋转平均使用四元数平均，避免 SO(3) 直接平均的病态问题）
5. 发布 TF（base_link → camera_optical_frame）
6. 若配置了 `calibration_file` 参数，自动保存到 YAML 文件

**标定结果持久化**：
- 设置 `calibration_file` 参数后，启动时自动加载已有标定结果
- 标定成功后自动保存到指定路径（YAML 格式，含旋转矩阵、四元数、位移向量）
- 重启后无需重新标定，直接进入对齐阶段

**订阅**：
- `/camera/image_raw` (Image) — ArUco 检测
- `/egm/robot_pose` (Pose) — 机器人TCP位姿

**发布**：
- TF: `base_link → camera_optical_frame`
- `/hand_eye_transform` (TransformStamped)

**服务**：
- `/calibrate` (Trigger) — 执行标定计算
- `/reset_samples` (Trigger) — 重置样本

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| aruco_marker_length | 0.05 | ArUco标记边长（m） |
| min_samples | 10 | 最少标定样本数 |
| camera_info_file | config/d415_camera_info.yaml | 相机内参 |

---

### 5.3 battery_pose_estimator

**功能**：将电池 4 角点像素坐标转换为 base_link 坐标系下的 3D 位姿。

**算法**：
1. 从 TF 获取 base_link → camera_optical_frame 变换
2. 使用 PnP（cv::solvePnP, IPPE）结合相机内参和 3D 模型点
   - IPPE（Iterative Pose Estimation）专为平面物体设计，比 ITERATIVE 更精确
   - 4种角点排列穷举，取重投影误差最小者
   - **重投影误差 > 10px 时拒绝该解**（2026-04-07 修复：原50px过大）；同时检查 Z 坐标物理合理性
3. 将电池角点从相机坐标系变换到 base_link 坐标系
4. 若相机内参检测到未标定占位符（fx>600 或 distortion 全零），运行时输出 WARN

**订阅**：`/battery_bboxes` (PoseArray)

**发布**：`/battery_poses` (PoseArray)

**3D 模型点（电池角点，单位mm）**：
```
(-35, -12.5, 0), (35, -12.5, 0), (35, 12.5, 0), (-35, 12.5, 0)
```

---

### 5.4 battery_alignment_controller

**功能**：PBVS 视觉伺服控制，将机器人 TCP 移动到电池正上方。

**控制逻辑**：
1. 获取电池在 base_link 下的位姿
2. 计算目标 TCP 位姿：电池中心 + 高度偏移（approach_height_m）
3. TCP 朝向：roll=0, pitch=π（朝下）, yaw=电池朝向角
4. 调用 MoveIt2 `/compute_ik` 获取关节目标
5. 发布 `/target_joint_states`

**状态机**：
```
IDLE → COMPUTING_IK → MOVING → COMPLETE
                              → ERROR（IK失败/超时）
ERROR ──新电池位姿到达──→ IDLE（自动恢复）
```

**并发保护**：
- 所有 shared state（`state_`、`battery_pose_`、`robot_pose_`、`has_*_flag` 等）均通过 `state_mutex_` 保护
- `pending_ik_request_` flag 防止并发 IK 请求重复发送
- `alignment_start_time_` 在 timer callback 中加锁读取
- `checkAlignmentTimeout` 完整加锁执行

**订阅**：
- `/battery_poses`
- `/egm/robot_pose`
- `/joint_states`

**发布**：
- `/target_joint_states`
- `/alignment_status` (Bool)
- `/desired_pose` (Pose, 调试用)

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| approach_height_m | 0.05 | 探头距电池表面高度 |
| alignment_timeout_sec | 2.0 | 对齐超时 |
| max_retry | 3 | IK最大重试次数 |
| alignment_threshold_m | 0.005 | 对齐完成位置误差阈值 |
| alignment_threshold_rad | 0.1 | 对齐完成姿态误差阈值（~5.7°） |
| ik_group_name | manipulator | MoveIt IK组名 |

---

### 5.5 cscan_trajectory_generator

**功能**：生成电池表面的蛇形栅格扫描路径，逐点触发 NI 超声采集。

**扫描路径**：zigzag 栅格（偶数行左→右，奇数行右→左）

**扫描流程**：
1. 接收 `/battery_poses` → 生成栅格点序列
2. 用户调用 `/cscan/start_scan` 触发扫描
3. 对每个栅格点：
   - 变换到 base_link 坐标系
   - 调用 `/compute_ik`
   - 发布 `/target_joint_states`
   - 发布 `/cscan_grid_trigger`（Int32 索引）
   - 等待 dwell_time_sec
4. 扫描完成 → `/cscan_status` = STATUS_COMPLETE

**网格计算示例**（默认参数）：
- nx = ceil(70mm / 1mm) = 70 点
- ny = ceil(25mm / 1mm) = 25 点
- 总计 = 1750 点

**订阅**：
- `/battery_poses`
- `/joint_states`

**发布**：
- `/cscan_grid_trigger` (Int32)
- `/cscan_status` (CscanStatus)

**服务**：
- `/cscan/start_scan` (Trigger)
- `/cscan/stop_scan` (Trigger)

**并发保护**：
- `battery_pose_` 在 `transformLocalToBaseLink` 中加锁 copy 后再使用
- `dwell_start_time_` 在 `timerCallback` 中加锁读取
- `checkIKTimeout` 完整加锁执行（pending flag + sent_time）
- `status_msg_` 所有写操作加锁保护
- `ik_failure_count_` 在 `handleIKResponse`、`moveToNextPoint`、`stopScanCallback` 中统一加锁

---

### 5.6 cscan_udp_bridge

**功能**：ROS2 与 NI 硬件之间的 UDP 双向桥接。

**三种运行模式**：

| 模式 | ni_ip | use_mock_ni | 行为 |
|------|-------|-------------|------|
| 真实硬件 | 非空 | false | 连接 NI 硬件，UDP 双向通信 |
| Mock 模式 | 任意 | true | 响应 /cscan_grid_trigger，生成模拟超声数据 |
| 降级模式 | **空** | false | 启动时进入降级模式，所有 callback 安全返回，不崩溃 |

> **注意**：当 ni_ip 为空且 use_mock_ni=false 时，节点进入降级模式（initialized_ok_=false），所有 callback（onGridTrigger、checkTimeout 等）检测到标志后静默返回，不崩溃、不报错。timeout timer 仍然运行但内部有检查。如需测试，请使用 `use_mock_ni:=true`。如需恢复，修正 ni_ip 参数后重启节点。

**数据流向**：
```
ROS2 /cscan_grid_trigger → UDP trigger → NI（触发采集）
NI UDP data → /ultrasonic_data → /ultrasonic_envelope
```

**Mock 模式说明**：
- 订阅 `/cscan_grid_trigger`（Int32 grid_index）
- 收到 trigger 后 100ms 内发布模拟 A-scan 数据
- 快速连续触发时（<100ms）生成高频噪声模拟真实采集延迟
- 用于无 NI 硬件时的完整流程测试

**NI UDP 协议**：

*发送（ROS2 → NI）*：
```
4 bytes (uint32 big-endian): grid_index
目标地址: ni_ip:ni_send_port (192.168.1.100:5001)
```

*接收（NI → ROS2）*：
```
12字节头部（大端序）：
  uint32 sample_count   // 采样点数
  uint32 timestamp_us  // 微秒时间戳
  uint32 grid_index    // 网格点索引
Payload: float32[sample_count] 归一化幅度值 (-1.0 ~ 1.0)
接收端口: ni_receive_port (5000)
```

**发布**：
- `/ultrasonic_data` (Float32MultiArray)
- `/ultrasonic_envelope` (Float32)
- `/cscan_ni_status` (Bool)
- `/cscan_packets_lost` (Int32) — 累计丢包数（NI UDP sequence number 跳变检测）

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| ni_ip | 192.168.1.100 | NI硬件IP地址 |
| ni_receive_port | 5000 | 接收端口 |
| ni_send_port | 5001 | 发送端口 |

---

### 5.7 cscan_visualizer

**功能**：实时构建 C-scan 图像和 A-scan 波形显示。

**C-scan 图像构建**：
1. 维护 ny_ × nx_ 的 CV_16UC1 缓冲区
2. 每个数据点到达时，将峰值幅度写入缓冲区对应位置
3. 使用 snake-like 路径索引（与轨迹生成器一致）
4. JET 伪彩色映射 + 状态叠加

**订阅**：
- `/ultrasonic_data`
- `/cscan_grid_trigger`
- `/cscan_status`
- `/joint_states`

**发布**：
- `/cscan_live_image` (Image) — 实时 C-scan 图像
- `/ascan_display` (Image) — A-scan 波形

**可视化元素**：
- JET 伪彩色 C-scan 图像
- 每 10 像素白色网格线
- 绿色当前位置点
- 进度文字（网格索引/总数/百分比）
- 状态文字（绿色=完成，红色=错误，黄色=扫描中）
- 关节位置显示（J1, J2, J3 角度）

---

## 6. NI UDP 通信协议

### 6.1 触发包格式（ROS2 → NI）

```
偏移    类型        大小        说明
0x00    uint32     4 bytes    grid_index（big-endian）
─────────────────────────────────
总大小：4 bytes
```

### 6.2 数据包格式（NI → ROS2）

```
偏移    类型        大小            说明
0x00    uint32     4 bytes    sample_count（big-endian）
0x04    uint32     4 bytes    timestamp_us（big-endian）
0x08    uint32     4 bytes    grid_index（big-endian）
0x0C    float32[]  N×4 bytes  采样数据（归一化幅度值）
─────────────────────────────────────────────
总大小：12 + N×4 bytes
最大：12 + 8192×4 = 32,780 bytes
```

### 6.3 常量定义

```cpp
constexpr size_t HEADER_SIZE = 12;        // 3 × uint32
constexpr size_t MAX_SAMPLE_COUNT = 8192; // 最大采样点数
constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;
```

---

## 7. 启动流程

### 7.1 环境准备

```bash
# 设置 ROS2 环境
source /opt/ros/humble/setup.bash
source /home/i/ros2_ws/install/setup.bash

# 编译（如需要）
cd /home/i/ros2_ws && colcon build
```

### 7.2 三个 bringup launch

#### 阶段一：手眼标定

```bash
ros2 launch abb_bringup abb_hand_eye_calibration.launch.py \
  robot_type:=gofa_crb15000_10_152 \
  robot_class:=gofa \
  sim:=false \
  use_real_camera:=true \
  camera_info_file:=share/abb_vision/config/d415_camera_info.yaml
```

**启动节点**：

| 时间 | 节点/组件 |
|------|---------|
| 0.0s | robot_state_publisher |
| 0.0s | joint_to_cartesian_pose_node（发布 /egm/robot_pose） |
| 0.0s | realsense2_camera（条件：use_real_camera=true） |
| 0.0s | hand_eye_calibrator |
| 0.0s | battery_detector（辅助验证） |

---

#### 阶段二：电池对齐

```bash
ros2 launch abb_bringup abb_alignment_bringup.launch.py \
  robot_type:=gofa_crb15000_10_152 \
  robot_class:=gofa \
  sim:=false \
  use_real_camera:=true \
  camera_info_file:=share/abb_vision/config/d415_camera_info.yaml
```

**启动节点**：

| 时间 | 节点/组件 | 条件 |
|------|---------|------|
| 0.0s | controller_manager | 无条件 |
| 0.0s | robot_state_publisher | 无条件 |
| 0.0s | joint_to_cartesian_pose_node | 无条件 |
| 0.0s | move_group（MoveIt2） | 无条件 |
| 0.5s | include_robot_pose | `use_mock_hand_eye=false` |
| 0.5s | mock_robot_pose_node + mock_camera_tf | `use_mock_hand_eye=true` |
| 1.0s | hand_eye_calibrator | `use_mock_hand_eye=false` |
| 1.0s | battery_detector | `use_mock_hand_eye=false` |
| 1.0s | battery_pose_estimator | `use_mock_hand_eye=false` |
| 2.0s | battery_alignment_controller | 无条件 |
| 2.0s | quintic_trajectory_planner | 无条件 |

---

#### 阶段三：C-scan 扫描

```bash
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  robot_type:=gofa_crb15000_10_152 \
  robot_class:=gofa \
  sim:=false \
  use_real_camera:=true \
  camera_info_file:=share/abb_vision/config/d415_camera_info.yaml \
  ni_ip:=192.168.1.100
```

**启动节点**（共 11 个组件）：

| 时间 | 节点/组件 | 条件 |
|------|---------|------|
| 0.0s | controller_manager | 无条件 |
| 0.0s | robot_state_publisher | 无条件 |
| 0.0s | joint_to_cartesian_pose_node | 无条件 |
| 0.0s | move_group | 无条件 |
| 0.5s | mock_data_node | `sim=true` |
| 0.5s | mock_robot_pose_node | `use_mock_hand_eye=true` |
| 0.5s | mock_camera_tf | `use_mock_hand_eye=true` |
| 1.0s | include_hand_eye | `use_mock_hand_eye=false` |
| 1.0s | battery_detector | `use_mock_hand_eye=false` |
| 1.0s | battery_pose_estimator | 无条件 |
| 2.0s | battery_alignment_controller | 无条件 |
| 2.0s | quintic_trajectory_planner | 无条件 |
| 3.0s | cscan_trajectory_generator | 无条件 |
| 3.0s | cscan_udp_bridge | 无条件 |
| 3.5s | cscan_visualizer | 无条件 |

---

## 8. 操作指南

### 8.1 操作流程总览

```
┌─────────────────────────────────────────────────────┐
│ 阶段一：手眼标定（一次性操作，标定结果持久化）        │
│                                                     │
│ 1. 启动标定 launch                                  │
│ 2. 将 ArUco 标定板固定在机器人末端工具               │
│ 3. 手动示教：移动机器人采集 ≥10 个不同位姿           │
│    - 建议移动距离 > 30cm，旋转角度 > 15°            │
│ 4. 调用标定服务：                                   │
│    ros2 service call /calibrate std_srvs/srv/Trigger "{}" │
│ 5. 验证 TF 发布正常：                               │
│    ros2 run tf2_tools view_frames                 │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 阶段二：电池对齐（每次换电池/换工件前执行）          │
│                                                     │
│ 1. 启动对齐 launch                                  │
│ 2. 等待 /alignment_status = true                   │
│    ros2 topic echo /alignment_status               │
│ 3. 如需手动触发对齐：移动相机对准电池表面后启动       │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 阶段三：C-scan 扫描                                 │
│                                                     │
│ 1. 启动 C-scan bringup                             │
│ 2. 等待 /cscan_status = STATUS_READY (2)            │
│ 3. 触发扫描：                                       │
│    ros2 service call /cscan/start_scan std_srvs/srv/Trigger "{}" │
│ 4. 监控进度：                                       │
│    ros2 topic echo /cscan_status                  │
│ 5. 查看实时 C-scan 图像：                           │
│    ros2 run image_view image_view --image /cscan_live_image │
│ 6. 扫描完成自动停止，或手动停止：                    │
│    ros2 service call /cscan/stop_scan std_srvs/srv/Trigger "{}" │
└─────────────────────────────────────────────────────┘
```

### 8.2 服务调用汇总

| 服务 | 类型 | 调用方式 |
|------|------|---------|
| `/calibrate` | Trigger | `ros2 service call /calibrate std_srvs/srv/Trigger "{}"` |
| `/reset_samples` | Trigger | `ros2 service call /reset_samples std_srvs/srv/Trigger "{}"` |
| `/cscan/start_scan` | Trigger | `ros2 service call /cscan/start_scan std_srvs/srv/Trigger "{}"` |
| `/cscan/stop_scan` | Trigger | `ros2 service call /cscan/stop_scan std_srvs/srv/Trigger "{}"` |

### 8.3 监控命令

```bash
# 查看所有活跃 topic
ros2 topic list

# 监控 C-scan 状态
ros2 topic echo /cscan_status

# 监控对齐状态
ros2 topic echo /alignment_status

# 监控电池位姿
ros2 topic echo /battery_poses

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看 /tf_static
ros2 topic echo /tf_static

# NI 连接状态
ros2 topic echo /cscan_ni_status

# 超声数据（峰值包络）
ros2 topic echo /ultrasonic_envelope
```

---

## 9. 参数配置

### 9.1 相机内参配置

相机内参文件（首次使用需要标定填入）：

**文件路径**：`/home/i/ros2_ws/src/abb_vision/config/d415_camera_info.yaml.template`

**标定方法**：
```bash
ros2 run camera_calibration cameracalibrator.py image:=/camera/image_raw camera:=/camera --no-service-check
```

**标定后填入内容示例**：
```yaml
camera_name: d455
camera_matrix:
  rows: 3
  cols: 3
  data: [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
image_width: 640
image_height: 480
```

### 9.2 电池尺寸参数

根据实际电池调整以下参数：

| 参数 | 文件 | 默认值 | 说明 |
|------|------|--------|------|
| battery_length_m | abb_cscan_bringup.launch.py | 0.07 | 电池长度（m） |
| battery_width_m | abb_cscan_bringup.launch.py | 0.025 | 电池宽度（m） |
| battery_length_mm | abb_vision 节点内 | 70.0 | 电池长度（mm） |
| battery_width_mm | abb_vision 节点内 | 25.0 | 电池宽度（mm） |

### 9.3 扫描参数

| 参数 | launch参数 | 默认值 | 说明 |
|------|-----------|--------|------|
| 扫描分辨率 | cscan_resolution_m | 0.001 (1mm) | 栅格间距 |
| 扫描速度 | cscan_speed_m_s | 0.005 | 扫描速度（未直接使用） |
| 探头高度 | cscan_approach_height_m | 0.05 | 探头距电池表面距离 |
| 驻留时间 | cscan_dwell_time_s | 0.05 | 每点停留时间（秒） |

### 9.4 NI 网络参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| ni_ip | （空） | NI 硬件 IP 地址（空=未配置） |
| use_mock_ni | false | true=使用 Mock NI 模拟器 |
| ni_receive_port | 5000 | ROS2 接收端口 |
| ni_send_port | 5001 | ROS2 发送端口 |

---

## 10. 三阶段工作流

### 10.1 阶段一：手眼标定详细流程

**前置条件**：
- RealSense D455 已连接并正常供电
- ArUco 标定板（6×6, DICT_6X6_250）已固定在工具末端
- 标定板参数已知（marker_length = 0.05m）

**操作步骤**：

1. **启动标定环境**
   ```bash
   ros2 launch abb_bringup abb_hand_eye_calibration.launch.py \
     use_real_camera:=true
   ```

2. **RViz 可视化（可选）**
   ```bash
   ros2 run rviz2 rviz2
   ```
   添加 TF 和 Image 插件，验证图像和 TF 正常。

3. **采集标定样本**
   - 手动示教移动机器人末端到不同位置
   - 每次移动后等待 1-2 秒
   - 目标：采集 ≥10 个样本，涵盖不同角度和位置
   - 采样覆盖建议：
     - 俯仰角 ±30°
     - 偏航角 ±45°
     - 位置 X/Y/Z 各方向移动 >30cm

4. **执行标定**
   ```bash
   ros2 service call /calibrate std_srvs/srv/Trigger "{}"
   ```

5. **验证结果**
   ```bash
   # 查看 /tf_static，应有 base_link → camera_optical_frame
   ros2 run tf2_ros tf2_echo base_link camera_optical_frame

   # 查看标定误差
   ros2 topic echo /hand_eye_transform
   ```

6. **重置样本（如需要重新标定）**
   ```bash
   ros2 service call /reset_samples std_srvs/srv/Trigger "{}"
   ```

**标定质量判断**：
- 重复标定 3 次，TF 结果一致性 < 5mm → 标定合格
- 误差过大 → 增加采样数量和角度覆盖范围

---

### 10.2 阶段二：电池对齐详细流程

**前置条件**：
- 手眼标定已完成且结果有效
- 电池已放置在相机视野范围内
- 机器人处于安全初始位置

**操作步骤**：

1. **启动对齐环境**
   ```bash
   ros2 launch abb_bringup abb_alignment_bringup.launch.py \
     use_real_camera:=true
   ```

2. **确认 TF 链路**
   ```bash
   # 应能查询到 base_link → camera_optical_frame
   ros2 run tf2_ros tf2_echo base_link camera_optical_frame
   ```

3. **确认相机图像**
   ```bash
   # 用 image_view 查看相机画面
   ros2 run image_view image_view --image /image
   ```
   应能看到橙色电池目标。

4. **确认电池检测输出**
   ```bash
   # 查看电池检测是否正常
   ros2 topic echo /battery_bboxes
   ```
   应输出 4 个角点 PoseArray 数据。

5. **确认电池位姿输出**
   ```bash
   ros2 topic echo /battery_poses
   ```
   应输出电池在 base_link 下的 3D 位姿。

6. **触发对齐（自动或手动）**
   - **自动模式**：电池在视野内，alignment_controller 自动启动对齐
   - **手动模式**：移动机器人使相机对准电池后，对齐控制器接管

7. **监控对齐状态**
   ```bash
   ros2 topic echo /alignment_status
   ```
   - `data: false` → 对齐中
   - `data: true` → 对齐完成

8. **对齐完成验证**
   - 机器人 TCP 移动至电池正上方
   - 探头距电池表面约 50mm
   - `/alignment_status = true`

---

### 10.3 阶段三：C-scan 扫描详细流程

**前置条件**：
- 阶段一和阶段二已完成
- NI 硬件已连接并配置（IP: 192.168.1.100），或使用 Mock 模式测试
- 超声探头已安装并对准电池表面（真实硬件模式）

**操作步骤**：

1. **启动 C-scan 环境**

   **方式一：使用真实 NI 硬件**
   ```bash
   ros2 launch abb_bringup abb_cscan_bringup.launch.py \
     use_real_camera:=true \
     ni_ip:=192.168.1.100
   ```

   **方式二：无 NI 硬件时使用 Mock 模式**
   ```bash
   ros2 launch abb_bringup abb_cscan_bringup.launch.py \
     use_real_camera:=true \
     use_mock_ni:=true
   ```

2. **确认对齐链路正常**
   ```bash
   ros2 topic echo /alignment_status
   # 应为 true，否则无法进入扫描阶段
   ```

3. **确认扫描状态就绪**
   ```bash
   ros2 topic echo /cscan_status
   # status 应为 2 (STATUS_READY)
   ```

4. **打开 C-scan 实时图像**
   ```bash
   ros2 run image_view image_view --image /cscan_live_image
   ```

5. **触发扫描**
   ```bash
   ros2 service call /cscan/start_scan std_srvs/srv/Trigger "{}"
   ```

6. **监控扫描进度**
   ```bash
   # 方法一：echo status
   ros2 topic echo /cscan_status

   # 方法二：echo NI 状态
   ros2 topic echo /cscan_ni_status
   ```

7. **CscanStatus 状态解读**

   | 值 | 状态名 | 说明 |
   |---|--------|------|
   | 0 | STATUS_IDLE | 空闲 |
   | 1 | STATUS_PATH_PLANNING | 路径规划中 |
   | 2 | STATUS_READY | 就绪（等待扫描触发）|
   | 3 | STATUS_SCANNING | 扫描中 |
   | 4 | STATUS_DWELL | 驻留（探头在当前点）|
   | 5 | STATUS_COMPLETE | 扫描完成 |
   | 6 | STATUS_ERROR | 错误 |

8. **扫描中手动停止（如遇异常）**
   ```bash
   ros2 service call /cscan/stop_scan std_srvs/srv/Trigger "{}"
   ```

9. **扫描完成后查看数据**
   ```bash
   # 保存 C-scan 图像
   ros2 run image_view image_view --image /cscan_live_image

   # A-scan 波形
   ros2 run image_view image_view --image /ascan_display
   ```

---

## 11. 故障排查

### 11.1 手眼标定故障

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| ArUco 标记未检测到 | 光照不足或标记放置角度过大 | 增加环境光照，调整标记角度 |
| 标定样本不足 | 机械臂移动范围不够 | 增加更多角度和位置的样本 |
| TF 未发布 | hand_eye_calibrator 未收到 /egm/robot_pose | 检查 joint_to_cartesian_pose_node 是否启动 |
| TF 精度差 | 样本采集角度重复性高 | 增加角度变化范围（>15°旋转） |

### 11.2 电池检测故障

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| 未检测到电池 | 橙色 HSV 阈值不合适 | 调整 battery_detector 的 HSV 参数 |
| 误检多 | 背景中有橙色物体 | 清理背景或调整 min_area |
| 角点输出抖动 | 图像噪声或电池表面反射 | 启用高斯模糊，调整 Canny 阈值 |

### 11.3 对齐控制故障

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| IK 服务调用失败 | MoveIt2 未正常启动 | 检查 move_group 是否在 launch 中启动 |
| 对齐超时 | 机械臂运动速度限制 | 增加 alignment_timeout_sec |
| /alignment_status 恒为 false | IK 求解失败 | 检查 /battery_poses 数据是否有效 |

### 11.4 C-scan 扫描故障

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| 扫描未触发 | /cscan_status 未达到 READY | 等待对齐完成，确认 battery_poses 有效 |
| NI 触发无响应 | NI IP 地址错误或网络不通 | 确认 ni_ip 参数，测试网络连通性 |
| UDP 数据丢失 | NI 端口配置错误 | 确认 ni_receive_port 和 ni_send_port |
| C-scan 图像全黑 | 无超声数据到达 | 检查 cscan_udp_bridge NI 连接状态 |
| 扫描路径偏移 | 电池位姿估计错误 | 重新执行手眼标定 |
| cscan_udp_bridge 启动报错退出 | ni_ip 为空且 use_mock_ni=false | 使用 `use_mock_ni:=true` 或配置正确 ni_ip |
| Mock 模式无数据 | Mock 依赖 /cscan_grid_trigger 触发 | 确认扫描已触发（/cscan_status = SCANNING） |

### 11.5 网络连通性测试

```bash
# 测试 NI 硬件连通性
ping 192.168.1.100

# 测试 ROS2 主机到机器人网桥
ping 192.168.125.100

# 检查 UDP 端口
sudo netstat -ulnp | grep 5000
sudo netstat -ulnp | grep 5001
```

---

## 12. 技术约束与已知问题

### 12.1 当前约束

| 约束项 | 说明 |
|--------|------|
| RealSense D455 驱动 | 需要 Ubuntu 22.04 + ROS2 Humble |
| ArUco 字典固定 | 当前使用 DICT_6X6_250，如需更换需修改代码 |
| 电池颜色假设 | HSV 橙色阈值针对特定电池型号设计 |
| GoFa 关节数量 | 硬编码 6 关节假设，不适用于其他臂型 |
| NI UDP 协议 | 12字节头部 + float32 payload 格式固定 |

### 12.2 已知限制

| 限制 | 说明 | 影响 |
|------|------|------|
| 手眼标定需手动示教 | 无法全自动批量标定 | 每次换相机需人工操作 |
| PnP 角点穷举 | 4 种排列穷举取最优，效率低 | 大批量检测时速度受限 |
| C-scan 图像 buffer 一次性 | 完成后 buffer 保留，不可重置 | 单次扫描后需重启节点 |
| 对齐控制器无重试恢复 | ERROR 状态后需重启节点 | 已修复：ERROR 状态可通过新电池位姿自动恢复到 IDLE |
| C-scan IK 失败时跳过继续 | IK 求解失败时自动跳至下一点 | 已修复：IK 失败后进入 ERROR 状态，连续失败 3 次（可配置）后彻底停止扫描 |
| STATUS_ERROR 消息类型未使用 | CscanStatus.msg 定义了 ERROR 但代码未主动设置 | 已修复：IK 失败、超时、服务异常均会设置 ERROR 状态 |
| abb_hardware_interface controller 切换崩溃 | on_deactivate 后 read/write 空指针崩溃 | 已修复：read/write 有空指针检查，on_deactivate 有 mutex 保护 |
| **abb_hardware_interface 缺失 on_configure()** | ros2_control 规范要求完整生命周期方法，缺失 on_configure() | 已修复(2026-04-07)：实现 on_configure() 生命周期方法，声明参数并验证 |
| **abb_hardware_interface export iface 无锁** | export_state_interfaces/export_command_interfaces 直接返回 motion_data_ 指针，并发 read/write 可能数据竞争 | 已修复(2026-04-07)：两函数均添加 motion_data_mutex_ 保护 |
| **abb_libegm EGMController callback 无锁** | EGMControllerInterface::callback() 无锁访问 configuration_.active，与 EGMTrajectoryInterface 模式不一致 | 已修复(2026-04-07)：锁内复制 configuration_.active 字段到局部变量，匹配 EGMTrajectoryInterface 模式 |
| **abb_rws_client polling_rate 无 inf 校验** | polling_rate<=0 有检查，但 inf 时 1000.0/inf=0 导致 wall_timer(0ms) 未定义 | 已修复(2026-04-07)：添加 std::isfinite 检查 |
| **abb_hand_eye_calibration launch NameError** | include_robot_pose 定义在使用之后(line 229 vs 182)，启动时 NameError | 已修复(2026-04-07)：将 include_robot_pose 定义移到 robot_pose_timer 引用之前 |
| **abb_alignment_bringup launch DeclareLaunchArgument 重复** | robot_ip 和 rws_port 各声明两次，启动报错 | 已修复(2026-04-07)：删除重复声明块 |
| **abb_vision PnP 误差阈值 50px** | 重投影误差 >50px 仅 DEBUG log，不拒绝错误解 | 已修复(2026-04-07)：阈值降至 10px，添加 Z 坐标物理合理性二次检查 |
| **battery_detector/pose_estimator cv::Exception 崩溃** | cv::FileStorage 构造函数在 YAML 格式不兼容时抛出 cv::Exception未被捕获，导致进程 exit -6 | 已修复(2026-04-08)：try-catch(cv::Exception) 包裹 fs.open()，解析失败时 fallback 到默认矩阵 |
| **abb_alignment_bringup battery节点条件缺失** | use_mock_hand_eye=true 时 battery_detector/pose_estimator 仍启动，与 mock_data_node 冲突（双重发布 /battery_bboxes） | 已修复(2026-04-08)：分离为独立 TimerAction，分别加 UnlessCondition/IfCondition |
| **abb_cscan_bringup battery_detector mock冲突** | battery_detector 在 use_mock_hand_eye=true 时被启动，与 mock_data_node 双重发布 /battery_bboxes | 已修复(2026-04-08)：battery_detector_node 加 condition=UnlessCondition(use_mock_hand_eye)，battery_pose_estimator 保持无条件 |
| **camera_info YAML 格式不兼容 OpenCV** | d415_camera_info.yaml.template 缺少 `%YAML:1.0` 头和 `!!opencv-matrix` 标签，OpenCV FileStorage 解析失败 | 运行时已通过 try-catch fallback 绕过（WARN 日志），节点使用默认矩阵；后续相机标定后替换为实际标定值（格式需改为 OpenCV 兼容格式） |
| **cscan_trajectory_generator 潜在死锁** | handleIKResponse 获取 state_mutex_ 后调用 updateStatus()（内部也需锁） | 已修复：updateStatus() 调用移至锁外 |
| **battery_detector HSV H 阈值过窄** | H=5-25 仅20度范围，光照变化时橙色分割失效 | 已修复(2026-04-07)：扩展到 0-40 + 自适应宽化机制(上限50) |
| **transformLocalToBaseLink 无四元数校验** | battery_pose_ 四元数无效时 tf2 产生 UB | 已修复(2026-04-07)：调用前检查四元数范数，无效时拒绝变换并告警 |
| **pending_ik_request_ 超时后状态不一致** | 超时清零 pending_ik_request_ 但无状态恢复 | 已修复(2026-04-07)：超时后转换到 ERROR 状态确保一致性 |
| **abb_alignment_bringup 缺失 launch 参数** | alignment_threshold_rad、joint_names 等参数未声明 | 已修复(2026-04-07)：补全 5 个 DeclareLaunchArgument |
| **URDF effort 占位值无说明** | GoFa effort=1000、IRB1300 effort=0 无注释，实际扭矩数据缺失 | 已修复(2026-04-07)：添加 NOTE 注释标注占位状态，ros2_control xacro 添加 effort iface 注释模板 |
| **D455→D415迁移** | `serial_no: '_t265_fisheye1'` 是T265跟踪相机序列号，D415使用不同设备标识 | 已修复：改为 `device_type:=D415`，launch文件d455→d415路径迁移 |
| **手眼标定结果不持久化** | 标定结果仅存在内存，重启后丢失 | 已修复：`calibration_file` 参数指定路径，启动自动加载，标定后自动保存 |
| **HSV阈值硬编码** | 橙色电池HSV参数在代码中写死，换电池颜色需改源码 | 已修复：6个HSV参数通过launch参数外部化，abb_alignment_bringup和abb_cscan_bringup均已配置 |
| **ExecuteProcess `command=` vs `cmd=`** | `abb_hand_eye_calibration.launch.py` 中 ExecuteProcess 使用 `command=` 作为参数名，但 ROS2 Humble 实际参数名是 `cmd`，导致 launch 加载时报 `missing 1 required keyword-only argument: 'cmd'` | 已修复(2026-04-07)：`command=` → `cmd=` |
| **robot_description 未用 ParameterValue 包装** | sim 模式下 robot_state_publisher 的 `robot_description` 参数直接传 xacro Command 输出字符串，rclpy YAML 解析器尝试将 XML 解析为 YAML 导致失败 | 已修复(2026-04-07)：`ParameterValue(robot_description_content_sim, value_type=str)` |
| **URDF/xacro关节限位错误** | joint_4/5±180°与实际物理限位±200°/±130°不符，velocity bounds全部缺失，MuJoCo无任何bounds | 已修复：ros2_control全面修正，新增多处velocity bounds，CRB15000 macro修正joint_2/3/4/5限位 |
| **irb1300 joint_2限位错误** | ros2_control中joint_2下限-155°，实际应为-95° | 已修复：-2.705→-1.658 |
| **abb_rws_client订阅数据竞争** | rws_state_publisher polling_rate=0导致除零崩溃；failure_count static int++数据竞争 | 已修复：polling_rate>0校验，failure_count改std::atomic<int> |
| **abb_egm_rws_managers TOCTOU** | debugText() const读system_data_无锁；isInterfaceReady try_lock后unlock再返回；description_锁外赋值 | 已修复：mutable+lock_guard；try_lock结果直接返回；description_移入mutex块 |
| **abb_libegm callback无锁** | EGMBaseInterface/EGMTrajectoryInterface callback中configuration_.active无锁访问 | 已修复：callback内加锁copy配置字段 |
| **abb_librws SubscriptionGroup double-close** | close()可能重复释放 | 已修复：swap实现幂等close |
| **quintic_trajectoryPlanner数据竞争** | target_joints在锁外被log访问；frame_count自增无锁；imageio.imwrite无异常处理 | 已修复：log移入锁内；帧计数加锁；imwrite加try/except |
| **cscan_udp_bridge未初始化+竞争** | 成员初始化列表未初始化；onReceive中ni_status_msg_数据竞争 | 已修复：声明处直接初始化；write和publish都在state_mutex_内 |
| **battery_detector除零** | 扫描起点边向量为零时norm()返回0导致除零 | 已修复：加norm零值检查 |
| **hand_eye_calibrator SO(3)平均病态** | 直接对SO(3)旋转矩阵元素平均，数学上无效 | 已修复：改用四元数平均（四元数算术平均后归一化） |
| **cscan_trajectory_generator死锁** | handleIKResponse中nested mutex调用moveToNextPoint→updateStatus | 已修复：unlock后再call updateStatus |
| **cscan_visualizer无锁+TOCTOU** | onJointState写joint_state_无锁；边界检查在锁外写入在锁内 | 已修复：全部加mutex；边界检查移入mutex块 |
| cscan_udp_bridge zombie 节点 | ni_ip 空且 mock=false 时构造函数提前返回，触发 callback 崩溃 | 已修复：initialized_ok_ flag 机制，所有 callback 有保护 |
| quintic_trajectory时间公式不安全 | T_acceleration = sqrt(d/a)*2，产出过短时间 | 已修复：物理正确的梯形 profile 公式 + 1.2 安全系数 |
| **EGM channel disconnect静默失败** | read()/write() 返回 OK 但 channel 已 inactive，高层节点无法感知 | 已修复：暴露 `egm_channel/active` state interface (double: 1.0=active, 0.0=inactive)，read()中日志警告 channel 变为 inactive |
| **joint name prefix stripping脆弱性** | `find("joint")` 字符串查找在 joint name 与预期不符时静默失败 | 已修复：移除 prefix stripping，joint name 直接从 motion_data_ (RWS/xacro) 使用；默认 prefix="" 配置下 joint_1/joint_2 等直接匹配 ros2_control xacro |
| trajectory_node 并发数据竞争 | 3 个回调并发访问共享状态无锁 | 已修复：threading.Lock 保护所有共享状态读写 |
| cscan_visualizer 数据竞争 | status_message_/progress_ 在不同 callback 间无保护 | 已修复：std::mutex 替换 recursive_mutex，所有共享变量读写加锁 |
| joint_state 越界访问 | joint_state.position.size() < 3 时仍访问 [0][1][2] | 已修复：访问前检查 size() >= 3 |
| battery_alignment_controller 数据争夺 | state_/has_*_flag/battery_pose_/robot_pose_/alignment_start_time_ 等 shared state 无 mutex 保护 | 已修复：全部 callback 和 timer 中所有 shared state 加 state_mutex_ 保护，添加 pending_ik_request_ 防重入 |
| cscan_trajectory_generator 数据争夺 | battery_pose_ (transformLocalToBaseLink 读 vs batteryPoseCallback 写)、dwell_start_time_、status_msg_、checkIKTimeout 等均无锁 | 已修复：transformLocalToBaseLink 入口加锁 copy battery_pose_；timerCallback 读 dwell_start_time_ 加锁；checkIKTimeout 整体加锁 |
| robot_type 默认值错误 | 所有 launch 文件 robot_type 默认为 irb1300_7_140，但实际应为 gofa_crb15000_10_152 | 已修复：5个 launch 文件全部更新默认值 |
| joint_trajectory 路由不一致 | abb_alignment_bringup 的 quintic_trajectory_planner 未 remap 到 /abb_controller/joint_trajectory | 已修复：补全 remap，与 abb_cscan_bringup 保持一致 |
| ik_timeout_sec 参数名不一致 | cpp 声明了 ik_timeout_sec 和 ik_service_timeout_sec 两个不同参数，实际只用到后者 | 已修复：移除 orphaned ik_timeout_sec param，统一使用 ik_timeout_sec param 映射到 ik_service_timeout_sec_ member |
| alignment_threshold_rad 未暴露 | cpp 有 alignment_threshold_rad 参数但 launch 未传递 | 已修复：abb_alignment_bringup 和 abb_cscan_bringup 均添加该参数 |
| joint_to_cartesian_pose_node 构造阻塞 | 构造函数中 rclcpp::sleep_for(100ms) 阻塞初始化 | 已修复：移除 sleep，publish_pose() 已有 tf2 exception handling |
| PnP 平面物体算法不最优 | 使用 cv::SOLVEPNP_ITERATIVE，对平面物体精度不如专用算法 | 已修复：升级为 cv::SOLVEPNP_IPPE（平面物体专用） |
| PnP 误差阈值不拒绝错误解 | 重投影误差 > 50px 时仅 DEBUG log，不拒绝解 | 已修复：error > 50px 时设置 found_valid=false 拒绝解 |

### 12.3 安全性注意

| 注意 | 说明 |
|------|------|
| 机械臂运动 | PBVS 对齐过程机械臂会运动，操作时保持安全距离 |
| EGM 实时控制 | 机械臂以 EGM 模式运行时，确保网络稳定 |
| NI 高压设备 | 超声设备含高压，操作时注意安全 |

---

## 附录

### A. 编译与构建

```bash
# 全量编译
cd /home/i/ros2_ws
colcon build

# 单独编译 abb_vision
colcon build --packages-select abb_vision

# 单独编译 abb_bringup
colcon build --packages-select abb_bringup

# 清理后重编
colcon build --cmake-clean-cache
```

### B. 验证可执行文件

```bash
ls /home/i/ros2_ws/install/abb_vision/lib/abb_vision/
# 应显示 7 个可执行文件：
# battery_alignment_controller  battery_detector
# battery_pose_estimator       cscan_trajectory_generator
# cscan_udp_bridge            cscan_visualizer
# hand_eye_calibrator
```

### C. 消息定义

**CscanStatus.msg**：
```msg
uint8 STATUS_IDLE = 0
uint8 STATUS_PATH_PLANNING = 1
uint8 STATUS_READY = 2
uint8 STATUS_SCANNING = 3
uint8 STATUS_DWELL = 4
uint8 STATUS_COMPLETE = 5
uint8 STATUS_ERROR = 6

uint8 status
uint32 current_point
uint32 total_points
float32 progress
string message
```

---

**文档版本**：v1.10
**更新日期**：2026-04-08
**更新内容**：Sprint 3 Review Round 2 Complete — cv::FileStorage cv::Exception未捕获(battery_detector/pose_estimator/hand_eye_calibrator)、abb_alignment_bringup battery节点条件缺失、abb_cscan_bringup battery_detector mock冲突、camera_info YAML格式不兼容OpenCV(有fallback)、全mock链路验证通过
**编写角色**：P9 Tech Lead
**适用版本**：Sprint 1 Complete + Sprint 2 Review Fixes + Sprint 3 Review Complete
