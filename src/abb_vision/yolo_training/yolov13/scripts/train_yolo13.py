#!/usr/bin/env python3
"""
YOLOv13 锂电池检测训练脚本

使用方法:
    python train_yolo13.py --data battery_dataset.yaml --epochs 100

依赖安装:
    pip install ultralytics torch torchvision
"""

import argparse
import os
import shutil
from pathlib import Path

def create_sample_dataset():
    """创建示例数据集目录结构"""
    base_dir = Path("datasets/battery")

    for split in ["train", "val", "test"]:
        (base_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (base_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    print(f"数据集目录已创建: {base_dir}")
    print("请将图像放入:")
    print("  - datasets/battery/images/train/")
    print("  - datasets/battery/images/val/")
    print("将标注文件(COCO或YOLO格式)放入:")
    print("  - datasets/battery/labels/train/")
    print("  - datasets/battery/labels/val/")

def download_public_dataset(dataset_name="battery"):
    """
    尝试从公开源下载数据集
    常用公开数据集源:
    - Roboflow Universe
    - Hugging Face
    - Kaggle
    """
    print(f"尝试下载公开数据集: {dataset_name}")
    print("=" * 50)
    print("推荐手动下载的数据集:")
    print()
    print("1. Roboflow Universe (推荐)")
    print("   - 搜索: 'lithium battery detection'")
    print("   - 网址: https://universe.roboflow.com/")
    print("   - 支持直接导出YOLO格式")
    print()
    print("2. Hugging Face Datasets")
    print("   - 搜索: 'battery detection'")
    print("   - 网址: https://huggingface.co/datasets")
    print()
    print("3. Kaggle Datasets")
    print("   - 搜索: 'battery segmentation'")
    print("   - 网址: https://www.kaggle.com/datasets")
    print()
    print("=" * 50)
    print()
    print("如果暂时没有数据集,可以:")
    print("1. 使用合成数据验证训练流程")
    print("2. 自行拍摄少量样本进行测试")

def train_yolo13(
    data_yaml,
    model_size="n",  # n/s/m/l/x
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,
    augment=True,
):
    """
    训练 YOLOv13 模型

    参数:
        data_yaml: 数据集配置文件路径
        model_size: 模型大小 (n/s/m/l/x)
        epochs: 训练轮数
        imgsz: 输入图像大小
        batch: 批次大小
        device: GPU设备号 (0,1,... 或 'cpu')
        augment: 是否使用数据增强
    """
    try:
        from ultralytics import YOLO
        import torch
    except ImportError as e:
        print(f"缺少依赖: {e}")
        print("请运行: pip install ultralytics torch torchvision")
        return

    # 模型选择
    model_name = f"yolov13{model_size}-seg.pt" if "seg" in str(model_size) else f"yolov13{model_size}.pt"
    print(f"模型选择: {model_name}")

    # 加载预训练模型 (网络不可用时跳过)
    print("尝试加载预训练模型...")
    try:
        model = YOLO("yolov13-seg.pt" if "seg" in str(model_size) else "yolov13.pt")
        print("预训练模型加载成功")
    except Exception as e:
        print(f"无法加载预训练模型: {e}")
        print("将使用随机初始化继续训练...")
        # 使用 ultralytics 提供的最新模型（本地已有或跳过）
        try:
            # 尝试 YOLOv8 作为替代（通常本地有缓存）
            model = YOLO("yolov8n-seg.pt")
            print("使用 YOLOv8n-seg 作为替代模型")
        except Exception as e2:
            print(f"YOLOv8 也无法加载: {e2}")
            print("请检查网络连接或手动下载模型")
            return None

    # 训练配置
    train_args = {
        "data": data_yaml,
        "epochs": epochs,
        "imgsz": imgsz,
        "batch": batch,
        "device": device,
        "project": "./runs",
        "name": "battery_detect",
        "exist_ok": True,
        "pretrained": True,
        "optimizer": "AdamW",
        "lr0": 0.01,
        "lrf": 0.01,
        "momentum": 0.937,
        "weight_decay": 0.0005,
        "warmup_epochs": 3,
        "warmup_momentum": 0.8,
        "close_mosaic": 10,
    }

    # 数据增强配置 (光照鲁棒)
    if augment:
        train_args.update({
            "mosaic": 1.0,          # Mosaic拼图增强
            "mixup": 0.15,          # MixUp增强
            "copy_paste": 0.15,     # Copy-paste增强
            "degrees": 15,          # 旋转范围
            "translate": 0.1,       # 平移
            "scale": 0.5,          # 缩放
            "fliplr": 0.5,         # 左右翻转
            "hsv_h": 0.02,         # 色调增强 ← 光照
            "hsv_s": 0.7,          # 饱和度增强 ← 光照
            "hsv_v": 0.5,          # 亮度增强 ← 光照
            "blur": 0.01,          # 模糊
            "clahe": 1.0,          # 直方图均衡 ← 光照关键
        })

    print("训练配置:")
    for k, v in train_args.items():
        print(f"  {k}: {v}")

    # 开始训练
    print("\n开始训练...")
    results = model.train(**train_args)

    # 保存最佳模型
    best_model_path = f"runs/battery_detect/weights/best.pt"
    if os.path.exists(best_model_path):
        shutil.copy(best_model_path, "models/best_yolo13_battery.pt")
        print(f"\n最佳模型已保存: models/best_yolo13_battery.pt")

    return results

def export_model(weights_path="models/best_yolo13_battery.pt", format="onnx"):
    """导出模型到指定格式"""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("请安装 ultralytics: pip install ultralytics")
        return

    model = YOLO(weights_path)
    export_path = model.export(format=format)
    print(f"模型已导出: {export_path}")
    return export_path

def main():
    parser = argparse.ArgumentParser(description="YOLOv13 锂电池检测训练")
    parser.add_argument("--data", type=str, default="datasets/battery_dataset.yaml",
                       help="数据集配置文件")
    parser.add_argument("--model", type=str, default="n",
                       choices=["n", "s", "m", "l", "x"],
                       help="模型大小: n=纳米, s=小, m=中, l=大, x=特大")
    parser.add_argument("--epochs", type=int, default=100,
                       help="训练轮数")
    parser.add_argument("--imgsz", type=int, default=640,
                       help="输入图像大小")
    parser.add_argument("--batch", type=int, default=16,
                       help="批次大小")
    parser.add_argument("--device", type=str, default="0",
                       help="训练设备 (0,1,... 或 cpu)")
    parser.add_argument("--no-augment", action="store_true",
                       help="禁用数据增强")
    parser.add_argument("--create-dataset", action="store_true",
                       help="创建数据集目录结构")
    parser.add_argument("--download-dataset", action="store_true",
                       help="显示数据集下载说明")
    parser.add_argument("--export", type=str, default=None,
                       help="导出模型格式 (onnx/tensorrt/tflite)")

    args = parser.parse_args()

    if args.create_dataset:
        create_sample_dataset()
        return

    if args.download_dataset:
        download_public_dataset()
        return

    if args.export:
        export_model(format=args.export)
        return

    # 检查数据集是否存在
    if not os.path.exists(args.data):
        print(f"数据集配置不存在: {args.data}")
        print("使用 --create-dataset 创建目录结构")
        create_sample_dataset()
        return

    # 训练模型
    train_yolo13(
        data_yaml=args.data,
        model_size=args.model,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        augment=not args.no_augment,
    )

if __name__ == "__main__":
    main()
