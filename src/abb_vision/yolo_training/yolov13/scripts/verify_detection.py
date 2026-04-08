#!/usr/bin/env python3
"""
YOLO13 检测验证脚本

用于验证训练好的模型在测试图像上的效果
"""

import cv2
import numpy as np
import argparse
from pathlib import Path

def draw_detections(image, results):
    """
    在图像上绘制检测结果

    Args:
        image: 输入图像 (BGR)
        results: YOLO检测结果
    """
    img = image.copy()

    for r in results:
        boxes = r.boxes
        masks = r.masks

        # 绘制检测框
        if boxes is not None:
            for box in boxes:
                # 获取边界框
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].cpu().numpy()
                cls = int(box.cls[0].cpu().numpy())

                # 绘制边界框
                color = (0, 255, 0)  # 绿色
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

                # 添加标签
                label = f"Class {cls}: {conf:.2f}"
                cv2.putText(img, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 绘制分割掩膜
        if masks is not None:
            # 为每个掩膜生成随机颜色
            colors = []
            np.random.seed(42)
            for _ in range(len(masks)):
                colors.append(tuple(np.random.randint(100, 255, 3).tolist()))

            for i, mask in enumerate(masks):
                # 获取掩膜
                m = mask.cpu().numpy()
                if isinstance(m, np.ndarray) and len(m.shape) == 2:
                    # 创建彩色掩膜
                    colored_mask = np.zeros_like(img)
                    colored_mask[m > 0.5] = colors[i % len(colors)]

                    # 叠加到原图
                    img = cv2.addWeighted(img, 0.7, colored_mask, 0.3, 0)

                    # 绘制边缘
                    contours, _ = cv2.findContours(
                        (m * 255).astype(np.uint8),
                        cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE
                    )
                    cv2.drawContours(img, contours, -1, colors[i % len(colors)], 2)

    return img

def test_on_image(model_path, image_path, output_path=None, conf=0.5):
    """测试单张图像"""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("请安装 ultralytics: pip install ultralytics")
        return

    # 加载模型
    print(f"加载模型: {model_path}")
    model = YOLO(model_path)

    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图像: {image_path}")
        return

    print(f"图像尺寸: {img.shape}")

    # 推理
    print("推理中...")
    results = model(img, conf=conf)

    # 绘制结果
    result_img = draw_detections(img, results)

    # 保存结果
    if output_path:
        cv2.imwrite(output_path, result_img)
        print(f"结果已保存: {output_path}")
    else:
        # 显示结果
        cv2.namedWindow("Detection Result", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detection Result", 800, 600)
        cv2.imshow("Detection Result", result_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # 打印检测统计
    for r in results:
        if r.boxes is not None:
            print(f"检测到 {len(r.boxes)} 个目标")
        if r.masks is not None:
            print(f"分割掩膜数量: {len(r.masks)}")

def test_on_camera(model_path, conf=0.5):
    """实时摄像头测试"""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("请安装 ultralytics: pip install ultralytics")
        return

    print(f"加载模型: {model_path}")
    model = YOLO(model_path)

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("按 'q' 退出，按 's' 保存当前帧")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 推理
        results = model(frame, conf=conf)

        # 绘制结果
        result_frame = draw_detections(frame, results)

        # 显示
        cv2.imshow("YOLO Detection", result_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"capture_{cv2.getTickCount()}.jpg"
            cv2.imwrite(filename, result_frame)
            print(f"已保存: {filename}")

    cap.release()
    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="YOLO13 检测验证")
    parser.add_argument("--model", type=str, default="models/best_yolo13_battery.pt",
                       help="模型路径")
    parser.add_argument("--image", type=str, default=None,
                       help="测试图像路径")
    parser.add_argument("--camera", action="store_true",
                       help="使用摄像头实时测试")
    parser.add_argument("--output", type=str, default=None,
                       help="输出图像路径")
    parser.add_argument("--conf", type=float, default=0.5,
                       help="置信度阈值")

    args = parser.parse_args()

    # 检查模型是否存在
    if not Path(args.model).exists():
        print(f"模型文件不存在: {args.model}")
        print("请先训练模型或下载预训练模型")
        return

    if args.camera:
        test_on_camera(args.model, args.conf)
    elif args.image:
        test_on_image(args.model, args.image, args.output, args.conf)
    else:
        print("请指定 --image 或 --camera")

if __name__ == "__main__":
    main()
