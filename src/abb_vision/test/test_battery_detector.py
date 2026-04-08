#!/usr/bin/env python3
"""
测试电池检测算法 - 生成合成测试图像
不需要实际相机，使用合成的锂电池图像验证检测流程
"""

import cv2
import numpy as np
import sys

def create_synthetic_battery_image(width=640, height=480):
    """创建合成锂电池测试图像"""
    image = np.ones((height, width, 3), dtype=np.uint8) * 200  # 灰色背景

    # 电池参数 (模拟 70mm x 25mm 锂电池)
    battery_width_px = 200
    battery_height_px = 70
    battery_angle = 15  # 旋转角度

    # 电池中心位置
    cx, cy = width // 2, height // 2

    # 创建电池区域 (橙色，模拟锂电池)
    battery_color = (30, 120, 220)  # BGR 橙色
    rect = ((cx, cy), (battery_width_px, battery_height_px), battery_angle)

    # 绘制旋转矩形 (电池主体)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(image, [box], 0, battery_color, -1)

    # 添加边缘高光
    cv2.drawContours(image, [box], 0, (40, 140, 240), 2)

    # 添加金属帽区域 (浅灰色，两端)
    cap_width = 15
    cap_height = battery_height_px - 10
    # 左金属帽
    left_cap_center = (cx - battery_width_px//2 - cap_width//2, cy)
    cv2.ellipse(image, (int(left_cap_center[0]), int(left_cap_center[1])),
               (cap_width, cap_height//2), 0, 0, 360, (180, 180, 190), -1)
    # 右金属帽
    right_cap_center = (cx + battery_width_px//2 + cap_width//2, cy)
    cv2.ellipse(image, (int(right_cap_center[0]), int(right_cap_center[1])),
               (cap_width, cap_height//2), 0, 0, 360, (180, 180, 190), -1)

    # 添加一些纹理
    for i in range(-3, 4):
        offset = i * (battery_height_px // 7)
        cv2.line(image,
                (cx - battery_width_px//2 + 20, cy + offset),
                (cx + battery_width_px//2 - 20, cy + offset),
                (25, 110, 210), 1)

    # 添加噪声模拟真实图像
    noise = np.random.normal(0, 5, image.shape).astype(np.int16)
    image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    return image, rect


def test_battery_detection():
    """测试电池检测算法"""
    print("=" * 60)
    print("锂电池检测算法测试")
    print("=" * 60)

    # 创建测试图像
    image, gt_rect = create_synthetic_battery_image(640, 480)
    ground_truth_center = (gt_rect[0][0], gt_rect[0][1])
    ground_truth_angle = gt_rect[2]

    print(f"\n[Ground Truth]")
    print(f"  电池中心: {ground_truth_center}")
    print(f"  旋转角度: {ground_truth_angle}°")

    # ========== 1. 预处理 ==========
    blurred = cv2.GaussianBlur(image, (5, 5), 1.5)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # ========== 2. 颜色分割 (橙色) ==========
    mask_orange = cv2.inRange(hsv, (5, 100, 100), (25, 255, 255))

    # 形态学操作
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # ========== 3. 边缘检测 ==========
    edges = cv2.Canny(mask, 50, 150, 3)

    # ========== 4. 轮廓提取 ==========
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print(f"\n[检测结果]")
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 1000:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter < 100:
            continue

        # 矩形拟合
        rect = cv2.minAreaRect(contour)
        rect_area = rect[1][0] * rect[1][1]
        solidity = area / rect_area if rect_area > 0 else 0

        # 筛选
        aspect_ratio = max(rect[1]) / min(rect[1]) if min(rect[1]) > 0 else 0
        expected_aspect = 70.0 / 25.0  # 2.8
        aspect_diff = abs(aspect_ratio - expected_aspect)

        print(f"  检测到轮廓: 面积={area:.0f}, 长宽比={aspect_ratio:.2f}, 矩形度={solidity:.2f}")

        if aspect_diff < 0.8 and solidity > 0.5:
            detections.append(rect)

    # ========== 5. 结果可视化 ==========
    vis_image = image.copy()

    if detections:
        for rect in detections:
            # 绘制检测框
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(vis_image, [box], 0, (0, 255, 0), 2)

            # 绘制中心点
            cv2.circle(vis_image, (int(rect[0][0]), int(rect[0][1])), 5, (0, 0, 255), -1)

            print(f"\n[最终检测]")
            print(f"  中心: ({rect[0][0]:.1f}, {rect[0][1]:.1f})")
            print(f"  尺寸: {rect[1][0]:.0f} x {rect[1][1]:.0f}")
            print(f"  角度: {rect[2]:.1f}°")

            # 与Ground Truth比较
            center_error = np.sqrt((rect[0][0] - ground_truth_center[0])**2 +
                                  (rect[0][1] - ground_truth_center[1])**2)
            print(f"  中心误差: {center_error:.1f} px")
    else:
        print("\n未检测到电池!")

    # 显示图像
    cv2.namedWindow("Synthetic Battery Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Synthetic Battery Image", 800, 600)
    cv2.imshow("Synthetic Battery Image", vis_image)

    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Mask", 800, 600)
    cv2.imshow("Mask", mask)

    print("\n按 'q' 退出...")
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

    return len(detections) > 0


if __name__ == "__main__":
    success = test_battery_detection()
    sys.exit(0 if success else 1)
