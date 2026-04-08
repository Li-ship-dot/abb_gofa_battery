#!/usr/bin/env python3
"""
测试 ArUco 标定板检测和手眼标定流程
使用合成图像验证算法
"""

import cv2
import numpy as np
import sys

def create_aruco_board_image():
    """创建 ArUco 标定板测试图像"""
    # ArUco 参数
    dict_name = cv2.aruco.DICT_6X6_250
    dictionary = cv2.aruco.getPredefinedDictionary(dict_name)

    # 标定板参数
    markers_x = 3
    markers_y = 3
    marker_length = 100  # 像素
    marker_separation = 20  # 标记间隔

    # 计算画布大小
    board_width = markers_x * (marker_length + marker_separation) + marker_separation
    board_height = markers_y * (marker_length + marker_separation) + marker_separation

    # 创建白色背景
    image = np.ones((board_height, board_width, 3), dtype=np.uint8) * 255

    # 生成标定板角点 (用于 drawCharUcoBoard)
    objPoints = []
    idPoints = []

    for y in range(markers_y):
        for x in range(markers_x):
            marker_id = y * markers_x + x
            if marker_id >= 25:  # DICT_6X6_250 有 250 个标记，我们用前25个
                break

            # 计算标记位置 (OpenCV 角点顺序)
            corners = np.array([
                [x * (marker_length + marker_separation) + marker_separation,
                 y * (marker_length + marker_separation) + marker_separation],
                [x * (marker_length + marker_separation) + marker_separation + marker_length,
                 y * (marker_length + marker_separation) + marker_separation],
                [x * (marker_length + marker_separation) + marker_separation + marker_length,
                 y * (marker_length + marker_separation) + marker_separation + marker_length],
                [x * (marker_length + marker_separation) + marker_separation,
                 y * (marker_length + marker_separation) + marker_separation + marker_length]
            ], dtype=np.float32)

            # 绘制标记
            cv2.aruco.drawMarker(dictionary, marker_id, marker_length, image,
                               int(corners[0][0]), thickness=1)

    return image, dictionary


def create_charuco_board_image():
    """创建 CharUco 标定板 (更精确的标定方法)"""
    dict_name = cv2.aruco.DICT_6X6_250
    dictionary = cv2.aruco.getPredefinedDictionary(dict_name)

    # CharUco 参数
    squares_x = 5
    squares_y = 7
    square_length = 80  # 棋盘格方块边长
    marker_length = 50  # ArUco 标记边长

    # 创建 CharUcoBoard
    board = cv2.aruco.CharucoBoard((squares_x, squares_y),
                                   float(square_length),
                                   float(marker_length),
                                   dictionary)

    # 创建图像
    board_width = squares_x * square_length
    board_height = squares_y * square_length
    image = np.ones((board_height, board_width, 3), dtype=np.uint8) * 255

    # 绘制 CharUco Board
    board.generateImage((board_width, board_height), image, 10, 1)

    return image, board


def test_aruco_detection():
    """测试 ArUco 标记检测"""
    print("=" * 60)
    print("ArUco 标定板检测测试")
    print("=" * 60)

    # 创建测试图像
    image, dictionary = create_aruco_board_image()

    print(f"\n[图像信息]")
    print(f"  尺寸: {image.shape[1]} x {image.shape[0]}")

    # 检测 ArUco 标记
    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary)

    print(f"\n[检测结果]")
    print(f"  检测到标记数: {len(ids) if ids is not None else 0}")
    print(f"  拒绝候选数: {len(rejected)}")

    if ids is not None:
        for i, marker_id in enumerate(ids):
            print(f"    标记 ID: {marker_id}, 角点数: {len(corners[i])}")

    # 绘制检测结果
    result_image = image.copy()
    cv2.aruco.drawDetectedMarkers(result_image, corners, ids)

    # 显示
    cv2.namedWindow("ArUco Board Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ArUco Board Detection", 900, 600)
    cv2.imshow("ArUco Board Detection", result_image)

    print("\n按 'q' 退出...")
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

    return len(ids) > 0


def test_charuco_detection():
    """测试 CharUco 角点检测"""
    print("=" * 60)
    print("CharUco 标定板检测测试")
    print("=" * 60)

    # 创建 CharUco 测试图像
    image, board = create_charuco_board_image()

    print(f"\n[图像信息]")
    print(f"  尺寸: {image.shape[1]} x {image.shape[0]}")
    print(f"  棋盘格: {board.getChessboardSize()}")

    # 检测
    corners, ids, rejected, recovered_ids = cv2.aruco.detectMarkers(image, board.dictionary)

    if len(corners) == 0:
        print("\n未检测到 ArUco 标记!")
        return False

    # 细化 CharUco 角点
    charuco_corners = []
    charuco_ids = []
    cv2.aruco.interpolateCornersCharuco(corners, ids, image, board, charuco_corners, charuco_ids)

    print(f"\n[检测结果]")
    print(f"  ArUco 标记数: {len(ids) if ids is not None else 0}")
    print(f"  CharUco 角点数: {len(charuco_corners)}")

    # 绘制结果
    result_image = image.copy()
    cv2.aruco.drawDetectedMarkers(result_image, corners, ids)
    if len(charuco_ids) > 0:
        cv2.aruco.drawDetectedCornersCharuco(result_image, charuco_corners, charuco_ids)

    # 估计位姿 (需要相机内参)
    camera_matrix = np.array([[600, 0, 320],
                              [0, 600, 240],
                              [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros(5)

    if len(charuco_ids) > 6:
        rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs)

        if rvec is not None:
            print(f"\n[位姿估计]")
            print(f"  旋转矢量: {rvec.flatten()}")
            print(f"  平移矢量: {tvec.flatten()}")

            # 绘制坐标轴
            cv2.drawFrameAxes(result_image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

    # 显示
    cv2.namedWindow("CharUco Board Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("CharUco Board Detection", 900, 600)
    cv2.imshow("CharUco Board Detection", result_image)

    print("\n按 'q' 退出...")
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

    return len(charuco_corners) > 0


def simulate_hand_eye_calibration():
    """模拟手眼标定数据收集"""
    print("=" * 60)
    print("手眼标定流程模拟")
    print("=" * 60)

    print("\n[手眼标定原理]")
    print("  目标: 计算 X = T_camera_robot (相机在机器人坐标系下的位姿)")
    print("  约束: A * X = X * B")
    print("    A = T_robot_base (机械臂末端相对基座的变换)")
    print("    B = T_marker_camera (标定板相对相机的变换)")
    print("")
    print("  算法: Tsai-Lenz (经典方法)")
    print("  步骤:")
    print("    1. 移动机械臂到 N 个不同姿态")
    print("    2. 在每个姿态记录机械臂位姿 (A_i)")
    print("    3. 同时检测标定板相对相机位姿 (B_i)")
    print("    4. 使用 N 组 (A_i, B_i) 计算 X")

    print("\n[标定流程]")
    print("  1. 安装标定板在相机视野内 (固定)")
    print("  2. 移动机械臂末端到标定板附近")
    print("  3. 在不同位置/角度下保持静止")
    print("  4. 系统自动记录数据")
    print("  5. 收集 >= 10 组样本后调用 /calibrate 服务")
    print("  6. 获得 T_base_camera 变换并发布为 TF")

    print("\n[输出]")
    print("  - TF: base_link -> camera_optical_frame")
    print("  - Topic: /hand_eye_transform (geometry_msgs/TransformStamped)")
    print("  - 可直接用于像素坐标到机器人坐标的转换")

    return True


if __name__ == "__main__":
    print("\n选择测试:")
    print("  1: ArUco 标记检测")
    print("  2: CharUco 角点检测")
    print("  3: 手眼标定原理说明")
    print("  4: 全部测试")

    choice = input("\n输入选择 (1-4): ").strip()

    if choice == '1':
        test_aruco_detection()
    elif choice == '2':
        test_charuco_detection()
    elif choice == '3':
        simulate_hand_eye_calibration()
    elif choice == '4':
        test_aruco_detection()
        test_charuco_detection()
        simulate_hand_eye_calibration()
    else:
        print("无效选择")
        sys.exit(1)
