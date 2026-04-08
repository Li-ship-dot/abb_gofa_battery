# 2026-04-05 Sprint 1 工作记录
# 锂电池超声非接触式检测 — ROS2 系统开发

## 项目背景

**目标**: 实现 ABB GoFa CRB15000 ROS2 驱动的锂电池超声非接触式检测系统
**硬件配置**:
- 机器人: ABB GoFa CRB15000 (10/152)
- 相机: Intel RealSense D455 (采购中)
- 超声探头: TecLab 空气耦合探头
- 超声收发: Ritec (已独立运行，不整合进 ROS2)
- 数据采集: NI 示波器/主机 (已独立运行)

---

## 阶段一: 项目分析

### 原有代码库分析 (`/home/i/ros2_ws`)

| 包 | 功能 | 状态 |
|-----|------|------|
| `abb_librws` | RWS/HTTP 通信库 | C++ |
| `abb_libegm` | EGM/UDP 实时运动控制 | C++ |
| `abb_egm_rws_managers` | EGM+RWS 综合管理 | C++ |
| `abb_ros2_msgs` | 消息/服务定义 (4子包) | msg/srv |
| `abb_hardware_interface` | ros2_control 硬件接口插件 | C++ |
| `abb_rws_client` | RWS 客户端节点 | C++ |
| `abb_vision` | 视觉系统 (电池检测/手眼标定) | C++ |
| `quintic_trajectory_planner` | 五次多项式轨迹规划 | Python |

**已有算法骨架，核心链路断裂**:
- 手眼标定内参硬编码为假值
- 无 `/egm/robot_pose` 发布节点
- 电池检测输出像素坐标，无坐标变换
- 无视觉伺服控制
- 无 C-scan 实现

---

## 阶段二: P9 任务拆解与执行

### 任务列表

| 优先级 | 任务 | 状态 | 执行人 |
|--------|------|------|--------|
| P0 | abb_vision 相机内参 YAML 化 | ✅ | P8-1 |
| P0 | `/egm/robot_pose` 发布节点 | ✅ | P8-1 |
| P0 | 手眼标定整合 launch | ✅ | P8-1 |
| P1 | 电池 3D 位姿估计 (PnP) | ✅ | P8-1 |
| P1 | PBVS 自动对齐控制器 | ✅ | P8-1 |
| P1 | 对齐整合 launch | ✅ | P8-1 |
| P2 | C-scan 轨迹生成器 | ✅ | P8-1 |
| P2 | C-scan 整合 launch | ✅ | P8-1 |
| P2 | NI ↔ ROS2 UDP 桥接节点 | ✅ | P8-2 (并行) |
| P2 | C-scan 实时可视化节点 | ✅ | P8-2 (并行) |
| P2 | C-scan bringup 整合 | ✅ | P9 |

---

## 交付物清单

### 新建/修改文件

#### abb_vision 包

| 文件 | 操作 | 说明 |
|------|------|------|
| `config/d455_camera_info.yaml.template` | 新建 | 相机内参 YAML 模板 |
| `src/hand_eye_calibrator.cpp` | 修改 | YAML 配置加载，移除硬编码内参 |
| `src/battery_detector.cpp` | 修改 | YAML 配置加载接口统一 |
| `launch/hand_eye_calibrator.launch.py` | 修改 | `camera_info_file` 参数 |
| `src/battery_pose_estimator.cpp` | 新建 | PnP 3D 位姿估计 |
| `launch/battery_pose_estimator.launch.py` | 新建 | 启动文件 |
| `src/battery_alignment_controller.cpp` | 新建 | PBVS 视觉伺服对齐 |
| `launch/battery_alignment_controller.launch.py` | 新建 | 启动文件 |
| `msg/CscanStatus.msg` | 新建 | 扫描状态消息 |
| `src/cscan_trajectory_generator.cpp` | 新建 | 栅格扫描轨迹生成 |
| `launch/cscan_trajectory_generator.launch.py` | 新建 | 启动文件 |
| `src/cscan_udp_bridge.cpp` | 新建 | NI ↔ ROS2 UDP 桥接 |
| `launch/cscan_udp_bridge.launch.py` | 新建 | 启动文件 |
| `src/cscan_visualizer.cpp` | 新建 | C-scan 实时可视化 |
| `launch/cscan_visualizer.launch.py` | 新建 | 启动文件 |
| `CMakeLists.txt` | 修改 | 添加所有节点编译目标 |

#### abb_omnicore_ros2 包

| 文件 | 操作 | 说明 |
|------|------|------|
| `abb_rws_client/src/joint_to_cartesian_pose_node.cpp` | 新建 | FK 发布 `/egm/robot_pose` |
| `abb_bringup/launch/abb_robot_pose.launch.py` | 新建 | 启动文件 |
| `abb_bringup/launch/abb_hand_eye_calibration.launch.py` | 新建 | 手眼标定整合 |
| `abb_bringup/launch/abb_alignment_bringup.launch.py` | 新建 | 对齐整合 |
| `abb_bringup/launch/abb_cscan_bringup.launch.py` | 新建 | C-scan 完整 bringup |

---

## 关键设计决策

### 1. 相机选型
**决策**: RealSense D455
**理由**: ROS2 官方驱动完善，深度精度 ~0.4mm @ 400mm，价格 $449，原型验证阶段最优选

### 2. 手眼标定断链
**问题**: `hand_eye_calibrator.cpp` 订阅 `/egm/robot_pose`，但无任何节点发布
**解决方案**: 新建 `joint_to_cartesian_pose_node`，通过 TF2 查 `base_link → tool0` 发布 Cartesian TCP pose

### 3. 视觉定位方案
**方案**: PnP (Perspective-n-Point) + 4角点
**输入**: `/battery_bboxes` (RotatedRect 4角点像素坐标)
**输出**: 电池在 `base_link` 下的 3D 位姿
**算法**: `cv::solvePnP` + 4种角点排列穷举取最优

### 4. 对齐控制方案
**类型**: PBVS (Position-Based Visual Servoing)
**链路**: `/battery_poses` + `/egm/robot_pose` → IK → `/target_joint_states` → quintic_trajectory_planner
**IK 服务**: MoveIt2 `/compute_ik` (group: `manipulator`)

### 5. C-scan 扫描方案
**路径**: 蛇形栅格 (zigzag)，避免空程
**同步**: `/cscan_grid_trigger` (Int32) 每网格点触发一次
**NI 对接**: UDP 桥接节点 (`cscan_udp_bridge`)
**可视化**: 实时 C-scan 图像 + A-scan 波形

---

## 完整系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        操作流程                                  │
│  手眼标定 → 对齐 → C-scan → NI 采集 → 后处理                    │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    消息/服务接口                                 │
│                                                                 │
│  /camera/image_raw           相机图像                            │
│  /battery_bboxes             电池4角点 (像素)                   │
│  /battery_detections         电池中心 (像素)                     │
│  /hand_eye_transform        手眼变换 (TransformStamped)         │
│  TF: base_link ↔ camera_optical_frame                          │
│  /egm/robot_pose             TCP Cartesian pose                  │
│  /battery_poses              电池 3D 位姿 (base_link)          │
│  /alignment_status           对齐完成 (Bool)                    │
│  /cscan_grid_trigger         网格触发 (Int32) ← NI 触发信号     │
│  /cscan_status               扫描状态 (CscanStatus)             │
│  /ultrasonic_data            A-scan 波形 (Float32MultiArray)    │
│  /ultrasonic_envelope        峰值幅度 (Float32)                 │
│  /cscan_live_image           C-scan 实时图像 (Image)            │
│  /ascan_display              A-scan 波形图 (Image)              │
│  /target_joint_states        关节目标 (JointState)             │
│  /joint_trajectory           平滑轨迹 (JointTrajectory)        │
│                                                                 │
│  /calibrate                  手眼标定触发 (Trigger)             │
│  /cscan/start_scan           开始扫描 (Trigger)                 │
│  /cscan/stop_scan            停止扫描 (Trigger)                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 启动命令

### 手眼标定
```bash
ros2 launch abb_bringup abb_hand_eye_calibration.launch.py \
  camera_info_file:=share/abb_vision/config/d455_camera_info.yaml \
  use_real_camera:=true
```

### 对齐
```bash
ros2 launch abb_bringup abb_alignment_bringup.launch.py \
  camera_info_file:=share/abb_vision/config/d455_camera_info.yaml \
  use_real_camera:=true
```

### C-scan 完整bringup
```bash
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  camera_info_file:=share/abb_vision/config/d455_camera_info.yaml \
  use_real_camera:=true \
  ni_ip:=192.168.1.100

# 触发扫描
ros2 service call /cscan/start_scan std_srvs/srv/Trigger "{}"

# 查看C-scan图像
ros2 run image_view image_view --image /cscan_live_image
```

---

## 相机到货后待办

| 序号 | 任务 | 说明 |
|------|------|------|
| 1 | 安装 RealSense D455 驱动 | `sudo apt install ros-humble-realsense2-camera` |
| 2 | 相机内参标定 | `ros2 run camera_calibration cameracalibrator.py` |
| 3 | 替换 `d455_camera_info.yaml.template` | 填入实际内参矩阵 |
| 4 | 手眼标定采集 | 10+ 样本，角度变化 > 15° |
| 5 | 验证 TF 精度 | 重复标定 3 次看一致性 |
| 6 | 对齐验证 | 对齐完成后 `/alignment_status=true` |
| 7 | C-scan 验证 | NI UDP 触发 + `/cscan_live_image` 可视化 |

---

## NI ↔ ROS2 UDP 协议格式

### 触发 (ROS2 → NI)
```
UDP packet: 4 bytes (uint32, network byte order)
  grid_index: 当前网格点索引
目标地址: ${ni_ip}:${ni_send_port}
```

### A-scan 数据 (NI → ROS2)
```
UDP packet:
  Header (12 bytes):
    uint32 sample_count    // A-scan 采样点数
    uint32 timestamp_us    // 微秒时间戳
    uint32 grid_index      // 网格点索引
  Payload:
    float32[sample_count]  // 归一化幅度值 (-1.0 ~ 1.0)
本地监听端口: ${ni_receive_port}
```

---

## 编译验证

```bash
# 全量编译
cd /home/i/ros2_ws && colcon build

# abb_vision 所有节点
ls install/abb_vision/lib/abb_vision/
  battery_alignment_controller
  battery_detector
  battery_pose_estimator
  cscan_trajectory_generator
  cscan_udp_bridge
  cscan_visualizer
  hand_eye_calibrator
```

---

## 遗留问题与风险

| 问题 | 风险等级 | 说明 |
|------|---------|------|
| RealSense D455 未到货 | 高 | 无法验证手眼标定 |
| PnP 角点排列依赖穷举 | 中 | 效率低但可靠 |
| GoFa 垂直向下 IK 解析解 | 中 | 目前用 MoveIt2 IK，服务可用性待验证 |
| NI UDP IP 地址硬编码 | 低 | 启动参数传入 |
| C-scan 图像 buffer 一次性 | 中 | 完成后保留，不可重置 |

---

## 参考资料

- `/home/i/ros2_ws/ROS2_架构与轨迹规划详解.md`
- `/home/i/ros2_ws/src/abb_vision/src/battery_detector.cpp`
- `/home/i/ros2_ws/src/abb_vision/src/hand_eye_calibrator.cpp`
- `/home/i/ros2_ws/src/abb_libegm/src/egm_udp_server.cpp` (boost::asio UDP 参考)
- MoveIt2 `moveit_msgs/srv/GetPositionIK` 服务

---

**记录日期**: 2026-04-05
**记录人**: P9 Tech Lead
**会话类型**: Sprint 1 规划与实现
