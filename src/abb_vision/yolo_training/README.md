# YOLO 锂电池检测训练框架

本目录包含两套 YOLO 训练框架，用于锂电池视觉检测任务。

## 目录结构

```
yolo_training/
├── yolov8/                      # YOLOv8 框架 (Ultralytics 官方)
│   ├── datasets/
│   │   └── battery_dataset.yaml
│   ├── scripts/
│   │   ├── train_yolov8.py     # 训练脚本
│   │   └── verify_detection.py  # 验证脚本
│   └── README.md
│
├── yolov13/                     # YOLOv13 框架 (学术机构)
│   ├── datasets/
│   │   └── battery_dataset.yaml
│   ├── scripts/
│   │   ├── train_yolo13.py     # 训练脚本
│   │   ├── create_synthetic_data.py  # 合成数据生成
│   │   └── verify_detection.py  # 验证脚本
│   └── README.md
│
├── datasets/                    # 共享数据集目录
│   └── battery/
│       ├── images/
│       │   ├── train/
│       │   └── val/
│       └── labels/
│           ├── train/
│           └── val/
│
├── models/                      # 模型保存目录
│   ├── best_yolov8_battery.pt
│   └── best_yolo13_battery.pt
│
└── README.md                    # 本文档
```
训练命令
YOLOv8:


cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov8
python scripts/train_yolov8.py --data datasets/battery_dataset.yaml --epochs 100
YOLOv13:


cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov13
python scripts/train_yolo13.py --data datasets/battery_dataset.yaml --epochs 100

## YOLOv8 vs YOLOv13 对比

| 特性 | YOLOv8 | YOLOv13 |
|------|--------|---------|
| **发布方** | Ultralytics 官方 | 清华大学等学术机构 |
| **稳定性** | ⭐⭐⭐⭐⭐ 最稳定 | ⭐⭐⭐⭐ |
| **社区资源** | 非常丰富 | 一般 |
| **部署成熟度** | 非常成熟 | 一般 |
| **TensorRT支持** | 完善 | 一般 |
| **推理速度** | 快 | 更快 |
| **精度** | 高 | 高 |

## 推荐选择

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   生产环境/工业部署: 推荐 YOLOv8                             │
│   - Ultralytics 官方维护                                    │
│   - 社区资源丰富，问题容易解决                               │
│   - TensorRT 部署案例多                                     │
│                                                             │
│   追求极致性能: 可尝试 YOLOv13                               │
│   - 精度略高                                                │
│   - 参数量更少                                              │
│   - 适合学术研究                                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 快速开始

### 1. 安装依赖

```bash
pip install ultralytics torch torchvision opencv-python numpy
```

### 2. 生成合成数据集 (验证流程)

```bash
cd /home/i/ros2_ws/src/abb_vision/yolo_training
python yolov13/scripts/create_synthetic_data.py --num-train 100 --num-val 20
```

### 3. 训练模型

**YOLOv8 (推荐):**
```bash
cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov8
python scripts/train_yolov8.py \
    --data datasets/battery_dataset.yaml \
    --epochs 100 \
    --model n
```

**YOLOv13:**
```bash
cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov13
python scripts/train_yolo13.py \
    --data datasets/battery_dataset.yaml \
    --epochs 100 \
    --model n
```

### 4. 验证检测

**YOLOv8:**
```bash
cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov8
python scripts/verify_detection.py \
    --model ../models/best_yolov8_battery.pt \
    --image test_image.jpg
```

**YOLOv13:**
```bash
cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov13
python scripts/verify_detection.py \
    --model ../models/best_yolo13_battery.pt \
    --image test_image.jpg
```

## 公开数据集获取

请手动下载以下公开数据集：

1. **Roboflow Universe** (推荐)
   - 搜索: `lithium battery`, `battery detection`
   - 网址: https://universe.roboflow.com/

2. **Hugging Face Datasets**
   - 搜索: `battery detection`
   - 网址: https://huggingface.co/datasets

3. **Kaggle**
   - 搜索: `battery segmentation`
   - 网址: https://www.kaggle.com/datasets

## 数据标注工具推荐

| 工具 | 特点 | 推荐度 |
|------|------|--------|
| Roboflow | 云端协作，自动增强 | ⭐⭐⭐⭐⭐ |
| Label Studio | 灵活，开源 | ⭐⭐⭐⭐ |
| CVAT | 强大，支持自动标注 | ⭐⭐⭐⭐ |
| Labelme | 简单，本地 | ⭐⭐⭐ |

## 下一步

1. 收集真实锂电池图像 (各型号 ≥50 张)
2. 使用标注工具进行标注
3. 使用本框架训练
4. 验证检测效果
5. 集成到 ROS2 机械臂控制链路


