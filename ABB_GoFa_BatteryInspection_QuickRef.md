# ABB GoFa 锂电池检测系统 — 快速操作指南

> 详细技术报告见 `ABB_GoFa_BatteryInspection_System_Report.md`

---

## 三阶段启动命令

> **工作流程**：阶段一标定一次，结果保存后阶段二/三启动时自动加载，无需重复标定。

### 阶段一：手眼标定（一次性操作）

```bash
ros2 launch abb_bringup abb_hand_eye_calibration.launch.py \
  robot_type:=gofa_crb15000_10_152 \
  robot_class:=gofa \
  sim:=false \
  use_real_camera:=true \
  calibration_file:=/home/i/ros2_ws/hand_eye_calibration.yaml

# 手动示教：移动机器人末端至不同位姿（建议 ≥10 个样本，位移 >30cm，旋转 >15°）
# 等待终端输出 "Sample N saved" 确认样本采集

# 执行标定
ros2 service call /calibrate std_srvs/srv/Trigger "{}"
# 成功后自动保存到 calibration_file

# 验证 TF 已发布
ros2 run tf2_ros tf2_echo base_link camera_optical_frame

# 如需重新标定：重置样本后重新采集
ros2 service call /reset_samples std_srvs/srv/Trigger "{}"
```

### 阶段二：电池对齐（每次换电池前执行）

```bash
ros2 launch abb_bringup abb_alignment_bringup.launch.py \
  robot_type:=gofa_crb15000_10_152 \
  robot_class:=gofa \
  sim:=false \
  use_real_camera:=true \
  calibration_file:=/home/i/ros2_ws/hand_eye_calibration.yaml

# 等待对齐完成
ros2 topic echo /alignment_status
# data: true → 对齐完成，可进入 C-scan
```

### 阶段三：C-scan 扫描

```bash
# 方式一：真实 NI 硬件
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  robot_type:=gofa_crb15000_10_152 \
  robot_class:=gofa \
  sim:=false \
  use_real_camera:=true \
  calibration_file:=/home/i/ros2_ws/hand_eye_calibration.yaml \
  ni_ip:=192.168.1.100

# 方式二：无 NI 硬件（Mock 模式，可完整测试全流程）
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  robot_type:=gofa_crb15000_10_152 \
  robot_class:=gofa \
  sim:=false \
  use_real_camera:=true \
  calibration_file:=/home/i/ros2_ws/hand_eye_calibration.yaml \
  use_mock_ni:=true

# 等待 /cscan_status = 2 (READY)
ros2 topic echo /cscan_status

# 触发扫描
ros2 service call /cscan/start_scan std_srvs/srv/Trigger "{}"

# 实时查看 C-scan 图像
ros2 run image_view image_view --image /cscan_live_image

# 手动停止（如遇异常）
ros2 service call /cscan/stop_scan std_srvs/srv/Trigger "{}"
```

---

## 监控命令速查

| 监控项 | 命令 |
|--------|------|
| 所有 topic | `ros2 topic list` |
| C-scan 状态 | `ros2 topic echo /cscan_status` |
| 对齐状态 | `ros2 topic echo /alignment_status` |
| 电池位姿 | `ros2 topic echo /battery_poses` |
| 超声 A-scan 数据 | `ros2 topic echo /ultrasonic_data` |
| 超声包络峰值 | `ros2 topic echo /ultrasonic_envelope` |
| NI 连接状态 | `ros2 topic echo /cscan_ni_status` |
| UDP 丢包计数 | `ros2 topic echo /cscan_packets_lost` |
| 网格触发索引 | `ros2 topic echo /cscan_grid_trigger` |
| TF 验证 | `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` |
| EGM 机器人位姿 | `ros2 topic echo /egm/robot_pose` |
| EGM channel 状态 | `ros2 topic echo /controller_manager/robot_state` (查找 `egm_channel.active`) |
| **HSV 自适应状态** | `ros2 topic echo /battery_detector/hsv_adaptive_status` |

---

## CscanStatus 状态值

| 值 | 状态 | 说明 |
|---|------|------|
| 0 | IDLE | 空闲 |
| 1 | PATH_PLANNING | 路径规划 |
| 2 | READY | 就绪，可触发扫描 |
| 3 | SCANNING | 扫描中（IK计算/移动中） |
| 4 | DWELL | 驻留当前点（等待 NI 采集） |
| 5 | COMPLETE | 扫描完成 |
| 6 | ERROR | 错误（IK失败等） |

---

## 关键参数（需根据实际电池调整）

| 参数 | launch 参数 | 默认值 |
|------|------------|--------|
| 电池长度 | `cscan_battery_length_m` | 0.07 (70mm) |
| 电池宽度 | `cscan_battery_width_m` | 0.025 (25mm) |
| 扫描分辨率 | `cscan_resolution_m` | 0.001 (1mm) |
| 探头高度 | `cscan_approach_height_m` | 0.05 (50mm) |
| 驻留时间 | `cscan_dwell_time_s` | 0.05 (50ms) |
| 对齐姿态阈值 | `alignment_threshold_rad` | 0.1 rad (~5.7°) |
| IK 超时 | `ik_timeout_sec` | 2.0 s |
| NI IP | `ni_ip` | （空，需手动配置） |
| Mock NI 模式 | `use_mock_ni` | false |
| NI 接收端口 | `ni_receive_port` | 5000 |
| NI 发送端口 | `ni_send_port` | 5001 |
| **HSV H min** | `hsv_h_min` | **0** （2026-04-07 更新）|
| **HSV H max** | `hsv_h_max` | **40** （2026-04-07 更新）|
| **HSV S min** | `hsv_s_min` | 100 |
| **HSV S max** | `hsv_s_max` | 255 |
| **HSV V min** | `hsv_v_min` | 100 |
| **HSV V max** | `hsv_v_max` | 255 |
| **标定结果文件** | `calibration_file` | （空） |

---

## 手眼标定结果持久化

```bash
# 1. 启动时指定保存路径（每次启动自动加载已有标定）
ros2 launch abb_bringup abb_hand_eye_calibration.launch.py \
  calibration_file:=/home/i/ros2_ws/hand_eye_calibration.yaml

# 2. 标定成功后自动保存到该路径
# 3. 下次启动时自动加载，无需重新标定
```

---

## HSV 自适应机制（2026-04-07 新增）

battery_detector 内置光照自适应，HSV 参数运行时可动态调整。

### 运行时动态调参

```bash
# 动态修改 HSV 参数（无需重启节点）
ros2 param set /battery_detector hsv_h_min 10
ros2 param set /battery_detector hsv_h_max 30
ros2 param set /battery_detector adaptive_failure_threshold 20

# 查看当前自适应状态
ros2 topic echo /battery_detector/hsv_adaptive_status
```

**hsv_adaptive_status 格式示例**：
```
adaptive_mode:NORMAL,current_h:[0,40],original_h:[0,40],limits:[0,50],at_limit:NO,consecutive_failures:0,consecutive_successes:5,last_detection:SUCCESS(1)
```

**自适应策略**：
- 连续 30 帧检测失败 → 自动宽化 H 范围（每次 -2/+2），上限 H=[0,50]
- 连续 60 帧检测成功 → 逐步收紧回原始范围

### 标定用 HSV 建议

黄色/橙色电池（默认参数）：
```bash
hsv_h_min:=0 hsv_h_max:=40 hsv_s_min:=100 hsv_s_max:=255 hsv_v_min:=100 hsv_v_max:=255
```

蓝色/黑色电池（示例）：
```bash
hsv_h_min:=100 hsv_h_max:=130 hsv_s_min:=50 hsv_s_max:=255 hsv_v_min:=30 hsv_v_max:=255
```

---

## Mock 硬件模式说明

用于无真实硬件时进行完整流程测试。三个参数独立控制：

| 参数 | 值 | 说明 |
|------|---|------|
| `sim` | true | 使用 mock ros2_control（不走真实 EGM） |
| `use_mock_hand_eye` | true | 无真实相机，mock_data_node 模拟 `/camera/image_raw` 和 `/battery_bboxes` |
| `use_mock_ni` | true | 无 NI 硬件，cscan_udp_bridge 内部切换到 mock 模式 |

**全 Mock 启动命令**：
```bash
ros2 launch abb_bringup abb_cscan_bringup.launch.py \
  sim:=true use_mock_hand_eye:=true use_mock_ni:=true
```

**Mock 模式数据流**：
```
mock_data_node (/joint_states 30Hz)
    → joint_to_cartesian_pose_node
    → mock_data_node (/battery_bboxes 2Hz)
    → battery_pose_estimator
    → /battery_poses
    → quintic_trajectory_planner
    → /target_joint_states
    → /abb_controller/joint_trajectory
```

**确认 Mock NI 模式生效**：
```bash
ros2 topic echo /cscan_ni_status
# data: true → Mock NI 模式已启用
```

---

## 轨迹路由路径

```
battery_alignment_controller
    /target_joint_states
           ↓
quintic_trajectory_planner （五次多项式平滑，per-joint velocity限幅）
    /abb_controller/joint_trajectory   ← 注意：不是 /joint_trajectory
           ↓
abb_controller （JointTrajectoryController）
           ↓
ABB GoFa OmniCore （EGM 实时控制）
```

**GoFa per-joint velocity limits（必须与 ros2_control xacro 一致）**：

| 关节 | velocity limit (rad/s) |
|------|----------------------|
| joint_1 | ±2.094 |
| joint_2 | ±2.09 |
| joint_3 | ±2.18 |
| joint_4/5/6 | ±3.49 |

> quintic_trajectory_planner 使用 `min(max_velocity, min(per_joint_limits))` 规划，确保不超过任何关节的物理上限。

---

## NI UDP 协议

**触发包（ROS2 → NI）**：4 bytes uint32 big-endian grid_index

**数据包（NI → ROS2）**：
```
12字节头 + float32[sample_count]
头: sample_count(4) + timestamp_us(4) + grid_index(4)
```

---

## 故障快速定位

| 现象 | 检查项 |
|------|--------|
| TF 未发布 | hand_eye_calibrator /calibrate 服务调用过？标定结果已保存？ |
| /battery_bboxes 空 | battery_detector 启动？相机画面正常？HSV 参数是否匹配电池颜色？ |
| /battery_poses 空 | 手眼 TF 发布正常？/battery_bboxes 有数据？相机内参已标定？ |
| /alignment_status 恒 false | MoveIt2 /compute_ik 服务正常？电池位姿有效？alignment_threshold_rad 阈值是否合适？ |
| C-scan 未触发 | /cscan_status = 2 (READY)？/alignment_status = true？ |
| NI 无数据 | ping 192.168.1.100 通吗？端口 5000/5001 被占用？ |
| cscan_udp_bridge 启动退出 | ni_ip 为空且 use_mock_ni=false → 使用 `use_mock_ni:=true` |
| Mock 模式无数据 | /cscan_grid_trigger 有无数据？Mock 依赖 trigger 触发 |
| /joint_trajectory 无效 | 确认路径是 `/abb_controller/joint_trajectory` 而非 `/joint_trajectory` |
| 电池位姿跳变 | 相机内参是否已标定？d415_camera_info.yaml 中 fx≈385 且 distortion 全零 = 未标定占位符，需重新标定 |
| PnP 解算失败率高 | PnP 算法已升级为 IPPE（平面物体专用）；若仍频繁失败需检查相机标定质量 |
| **ArUco 相机启动失败** | D455→D415迁移后，确认 realsense2_camera 已安装，且 `device_type:=D415` 参数正确 |
| **EGM channel inactive 告警** | abb_hardware_interface 日志出现 "EGM channel became inactive" → 机器人EGM连接中断，检查网线/EGM配置；/controller_manager/robot_state 中 egm_channel.active=0.0 确认 |
| **joint name 不匹配** | ros2_control xacro 与 motion_data_ joint name 必须一致；默认 prefix="" 配置下均为 joint_1/joint_2...，无需 prefix stripping |

---

## 构建验证

```bash
# 编译所有包
colcon build

# 验证节点可执行文件
ls install/abb_vision/lib/abb_vision/
# 应显示：battery_detector battery_pose_estimator hand_eye_calibrator
#         battery_alignment_controller cscan_trajectory_generator
#         cscan_udp_bridge cscan_visualizer

# 验证 abb_robot_pose 可用
ls install/abb_rws_client/lib/abb_rws_client/
# 应显示：joint_to_cartesian_pose_node rws_client

# 运行 launch smoke test（需先 source）
python3 -m pytest src/abb_omnicore_ros2/abb_bringup/test/ --collect-only
```

---

## 版本信息

**v1.8** | 2026-04-07 | Sprint 3 Review Complete：launch P0修复×2(hinclude_robot_pose顺序/DeclareLaunchArgument重复)、hardware_interface on_configure生命周期+deactivate崩溃+mutex+exportiface锁、EGM/RWS线程安全×3、abb_vision P0/P1×5、URDF effort规范化、CI/CD建立、launch test×3、HSV自适应机制

**v1.10** | 2026-04-08 | Sprint 3 Review Round 2 Complete：cv::FileStorage cv::Exception未捕获导致battery节点崩溃(try-catch修复)；abb_alignment_bringup battery节点条件缺失；abb_cscan_bringup battery_detector mock模式冲突；全mock链路(sim+use_mock_hand_eye+use_mock_ni)验证通过

**v1.10** | 2026-04-08 | Sprint 3 Review Round 2 Complete：cv::FileStorage cv::Exception未捕获导致battery节点崩溃；abb_alignment_bringup/abb_cscan_bringup battery节点条件缺失；全mock链路验证通过

**v1.9** | 2026-04-07 | Sprint 3 Review Round 2 Fixes：launch smoke test发现P0×2(ExecuteProcess `command=`→`cmd=`、robot_description需ParameterValue包装)，全链路sim启动验证通过

**v1.7** | 2026-04-07 | Sprint 3 Review Round 2：EGM channel state暴露(egm_channel/active)、prefix stripping移除

**v1.5** | 2026-04-06 | D455→D415相机迁移、标定持久化(calibration_file)、HSV 6参数全外部化、URDF/xacro joint limits全面修正

**v1.4** | 2026-04-06 | Sprint 2 Review：robot_type默认值修正、tf2::TimePointZero→tf2::TimePoint()、timer初始化、off-by-one修复、PnP→IPPE、PnP误差阈值修正、数据争夺10处全修复

**v1.3** | 2026-04-05 | Sprint 1 Review：launch类型修正、EGM空指针保护、ERROR状态恢复、IK失败保护、UDP丢包检测

**v1.2** | 2026-04-05 | Round 4 Fix：abb_robot_pose语法修复、Mock NI说明

**v1.1** | 2026-04-05 | Round 3 Fix：Mock模式、/joint_trajectory路由修正

**v1.0** | 2026-04-05 | 初始版本
