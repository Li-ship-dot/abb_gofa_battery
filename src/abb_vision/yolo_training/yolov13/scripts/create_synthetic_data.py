#!/usr/bin/env python3
"""
合成锂电池图像生成器 - 用于验证训练流程

生成模拟工业环境下的锂电池图像，包含:
- 不同光照条件
- 不同角度
- 不同尺寸
- 噪声和模糊

注意: 这只是用于验证训练流程，真实场景需要真实标注数据
"""

import cv2
import numpy as np
import os
from pathlib import Path
import random

class SyntheticBatteryGenerator:
    """合成锂电池图像生成器"""

    def __init__(self, output_dir="datasets/battery"):
        self.output_dir = Path(output_dir)
        self.img_dir = self.output_dir / "images"
        self.label_dir = self.output_dir / "labels"

        # 创建目录
        for split in ["train", "val"]:
            (self.img_dir / split).mkdir(parents=True, exist_ok=True)
            (self.label_dir / split).mkdir(parents=True, exist_ok=True)

        # 电池颜色范围 (模拟不同光照下的橙色电池)
        self.battery_colors = [
            ((5, 100, 100), (25, 255, 255)),    # 正常光照
            ((0, 80, 80), (30, 255, 255)),       # 暗光
            ((10, 60, 60), (35, 255, 255)),      # 强光
            ((0, 50, 50), (20, 200, 200)),       # 低饱和度
        ]

        # 电池尺寸 (像素)
        self.battery_sizes = [
            (150, 50),   # 长方形
            (120, 40),
            (180, 60),
        ]

    def generate_battery_image(self, width=640, height=480, num_batteries=1):
        """
        生成单张含锂电池的图像

        返回: (image, batteries_info)
            - image: BGR图像
            - batteries_info: list of (cx, cy, w, h, angle) 像素坐标
        """
        # 创建背景 (模拟工业环境)
        bg_color = random.randint(180, 220)
        image = np.ones((height, width, 3), dtype=np.uint8) * bg_color

        # 添加背景纹理
        noise = np.random.normal(0, 5, image.shape).astype(np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        batteries_info = []

        for _ in range(num_batteries):
            # 随机电池参数
            color_range = random.choice(self.battery_colors)
            size = random.choice(self.battery_sizes)
            w, h = size
            angle = random.uniform(-30, 30)
            cx = random.randint(w, width - w)
            cy = random.randint(h, height - h)

            # 绘制电池
            mask, bgr_color = self.draw_battery(
                width, height, cx, cy, w, h, angle, color_range
            )

            # 叠加到背景
            image[mask > 0] = bgr_color

            # 记录电池信息 (用于生成标签)
            batteries_info.append((cx, cy, w, h, angle))

        # 添加光照变化
        image = self.apply_lighting(image)

        return image, batteries_info

    def draw_battery(self, width, height, cx, cy, w, h, angle, color_range):
        """绘制旋转矩形电池"""
        # 创建掩膜
        mask = np.zeros((height, width), dtype=np.uint8)

        # 旋转矩形
        rect = ((cx, cy), (w, h), angle)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # 绘制填充矩形到掩膜
        cv2.fillPoly(mask, [box], 255)
        cv2.drawContours(mask, [box], 0, 255, 2)

        # 随机颜色
        hsv_color = (
            random.randint(color_range[0][0], color_range[1][0]),
            random.randint(color_range[0][1], color_range[1][1]),
            random.randint(color_range[0][2], color_range[1][2]),
        )
        bgr_color = tuple(map(int, cv2.cvtColor(
            np.array([[hsv_color]], dtype=np.uint8), cv2.COLOR_HSV2BGR
        )[0, 0]))

        # 添加高光模拟金属帽
        cap_width = 15
        cap_height = h - 10
        left_cap_center = (cx - w//2 - cap_width//2, cy)
        right_cap_center = (cx + w//2 + cap_width//2, cy)
        cv2.ellipse(mask, (int(left_cap_center[0]), int(left_cap_center[1])),
                    (cap_width, cap_height//2), 0, 0, 360, 255, -1)
        cv2.ellipse(mask, (int(right_cap_center[0]), int(right_cap_center[1])),
                    (cap_width, cap_height//2), 0, 0, 360, 255, -1)

        # 创建彩色电池图像
        battery_img = np.full((height, width, 3), bgr_color, dtype=np.uint8)

        return mask, bgr_color

    def apply_lighting(self, image):
        """应用随机光照效果"""
        # 亮度变化
        brightness = random.uniform(0.7, 1.3)
        image = np.clip(image * brightness, 0, 255).astype(np.uint8)

        # 添加阴影
        if random.random() > 0.5:
            shadow = random.uniform(0.8, 1.0)
            h, w = image.shape[:2]
            mask = np.random.rand(h, w) > 0.5
            image[mask] = np.clip(image[mask] * shadow, 0, 255).astype(np.uint8)

        # 添加镜头畸变模拟
        if random.random() > 0.7:
            k1 = random.uniform(-0.001, 0.001)
            k2 = random.uniform(-0.001, 0.001)
            h, w = image.shape[:2]
            k = np.array([[1 + k1, 0, 0], [0, 1 + k2, 0], [0, 0, 1]])
            image = cv2.warpPerspective(image, k, (w, h))

        return image

    def save_yolo_label(self, img_width, img_height, batteries_info, label_path):
        """保存YOLO格式标注"""
        with open(label_path, 'w') as f:
            for cx, cy, w, h, angle in batteries_info:
                # 转换为YOLO格式 (归一化中心点 + 宽高)
                x_center = cx / img_width
                y_center = cy / img_height
                box_width = w / img_width
                box_height = h / img_height

                # YOLO格式: class_id x_center y_center width height
                # class_id = 0 (锂电池)
                f.write(f"0 {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}\n")

    def generate_dataset(self, num_train=100, num_val=20):
        """生成完整数据集"""
        print("=" * 50)
        print("生成合成锂电池数据集")
        print("=" * 50)

        # 生成训练集
        print(f"\n生成训练集: {num_train} 张图像")
        for i in range(num_train):
            img, batteries_info = self.generate_battery_image(
                num_batteries=random.randint(1, 3)
            )

            # 保存图像
            img_path = self.img_dir / "train" / f"battery_{i:04d}.jpg"
            cv2.imwrite(str(img_path), img)

            # 保存标注
            label_path = self.label_dir / "train" / f"battery_{i:04d}.txt"
            self.save_yolo_label(img.shape[1], img.shape[0], batteries_info, label_path)

            if (i + 1) % 20 == 0:
                print(f"  已生成: {i + 1}/{num_train}")

        # 生成验证集
        print(f"\n生成验证集: {num_val} 张图像")
        for i in range(num_val):
            img, batteries_info = self.generate_battery_image(
                num_batteries=random.randint(1, 2)
            )

            img_path = self.img_dir / "val" / f"battery_{i:04d}.jpg"
            cv2.imwrite(str(img_path), img)

            label_path = self.label_dir / "val" / f"battery_{i:04d}.txt"
            self.save_yolo_label(img.shape[1], img.shape[0], batteries_info, label_path)

        print("\n" + "=" * 50)
        print(f"数据集生成完成!")
        print(f"图像目录: {self.img_dir}")
        print(f"标注目录: {self.label_dir}")
        print("=" * 50)

        # 显示数据集统计
        train_images = len(list((self.img_dir / "train").glob("*.jpg")))
        val_images = len(list((self.img_dir / "val").glob("*.jpg")))
        print(f"\n数据集统计:")
        print(f"  训练集: {train_images} 张图像")
        print(f"  验证集: {val_images} 张图像")

        return str(self.output_dir)

def main():
    import argparse
    parser = argparse.ArgumentParser(description="生成合成锂电池数据集")
    parser.add_argument("--output", type=str, default="datasets/battery",
                       help="输出目录")
    parser.add_argument("--num-train", type=int, default=100,
                       help="训练集图像数量")
    parser.add_argument("--num-val", type=int, default=20,
                       help="验证集图像数量")

    args = parser.parse_args()

    generator = SyntheticBatteryGenerator(args.output)
    dataset_path = generator.generate_dataset(args.num_train, args.num_val)

    print(f"\n数据集路径: {dataset_path}")
    print("\n接下来运行训练:")
    print(f"  python scripts/train_yolo13.py --data {dataset_path}/battery_dataset.yaml --epochs 100")

if __name__ == "__main__":
    main()
