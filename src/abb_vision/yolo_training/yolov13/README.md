# YOLOv13 锂电池检测训练框架

## 目录结构

```
yolov13/
├── datasets/
│   └── battery_dataset.yaml    # 数据集配置
├── scripts/
│   ├── train_yolo13.py       # 训练脚本
│   ├── create_synthetic_data.py  # 合成数据生成
│   └── verify_detection.py   # 检测验证
└── README.md
```

## 快速开始

### 1. 安装依赖

```bash
pip install ultralytics torch torchvision
```

### 2. 生成合成数据集 (验证流程)

```bash
cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov13
python scripts/create_synthetic_data.py --num-train 100 --num-val 20
```

### 3. 训练模型

```bash
cd /home/i/ros2_ws/src/abb_vision/yolo_training/yolov13
python scripts/train_yolo13.py \
    --data datasets/battery_dataset.yaml \
    --epochs 100 \
    --model n \
    --imgsz 640 \
    --batch 16
```

### 4. 验证检测

```bash
python scripts/verify_detection.py \
    --model models/best_yolo13_battery.pt \
    --image test_image.jpg
```

## 公开数据集获取

### 推荐数据集源

1. **Roboflow Universe** (推荐)
   - 搜索关键词: `lithium battery`, `battery detection`, `industrial battery`
   - 网址: https://universe.roboflow.com/
   - 支持直接导出 YOLO 格式

2. **Hugging Face Datasets**
   - 搜索关键词: `battery detection`, `industrial inspection`
   - 网址: https://huggingface.co/datasets

3. **Kaggle**
   - 搜索关键词: `battery segmentation`, `defect detection`
   - 网址: https://www.kaggle.com/datasets

## 模型参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--model` | n | 模型大小: n(纳米)/s(小)/m(中)/l(大)/x(特大) |
| `--epochs` | 100 | 训练轮数 |
| `--imgsz` | 640 | 输入图像大小 |
| `--batch` | 16 | 批次大小 |
| `--device` | 0 | GPU设备号 |

## 数据增强配置 (光照鲁棒)

```python
# 光照相关增强
"hsv_h": 0.02   # 色调变化 ±2%
"hsv_s": 0.7    # 饱和度变化 ±70%
"hsv_v": 0.5    # 亮度变化 ±50%
"clahe": 1.0    # 自适应直方图均衡

# 几何增强
"mosaic": 1.0   # Mosaic拼图
"mixup": 0.15   # MixUp混合
"degrees": 15    # 旋转 ±15°
```

## YOLOv8 vs YOLOv13 对比

| 特性 | YOLOv8 | YOLOv13 |
|------|--------|---------|
| 发布方 | Ultralytics 官方 | 清华大学等学术机构 |
| 稳定性 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| 社区资源 | 非常丰富 | 一般 |
| 部署成熟度 | 非常成熟 | 一般 |
| 精度 | 高 | 高 |
