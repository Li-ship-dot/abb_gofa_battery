#!/usr/bin/env python3
"""
Mock Data Node - Simulates hardware inputs for full simulation chain testing.

Publishes:
  /joint_states          - Fake robot joint states (6 joints, 30Hz)
  /camera/image_raw       - Fake camera images (orange rectangle, 5Hz)
  /battery_bboxes         - Fake battery bounding box as PoseArray (2Hz)

Usage:
    ros2 run abb_vision mock_data_node
"""

import math
import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np


class MockDataNode(Node):
    def __init__(self):
        super().__init__('mock_data_node')

        self.declare_parameter('publish_rate_joints', 30.0)
        self.declare_parameter('publish_rate_camera', 5.0)
        self.declare_parameter('publish_rate_bboxes', 2.0)

        self.rate_joints = self.get_parameter('publish_rate_joints').value
        self.rate_camera = self.get_parameter('publish_rate_camera').value
        self.rate_bboxes = self.get_parameter('publish_rate_bboxes').value

        self.bridge = CvBridge()
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 5)
        self.bbox_pub = self.create_publisher(PoseArray, '/battery_bboxes', 5)

        self.start_time = time.time()
        self.joint_names = [f'joint_{i+1}' for i in range(6)]
        self.frame_id = 0

        self.get_logger().info(
            f'Mock Data Node started: joints={self.rate_joints}Hz, '
            f'camera={self.rate_camera}Hz, bboxes={self.rate_bboxes}Hz'
        )

    def publish_joints(self):
        t = time.time() - self.start_time
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            0.5 * math.sin(t * 0.3 + 0.0),
            0.3 * math.sin(t * 0.5 + 1.0),
            0.4 * math.sin(t * 0.4 + 2.0),
            0.2 * math.sin(t * 0.6 + 3.0),
            0.3 * math.sin(t * 0.35 + 4.0),
            0.1 * math.sin(t * 0.45 + 5.0),
        ]
        msg.velocity = [0.0] * 6
        self.joint_pub.publish(msg)

    def publish_camera(self):
        img = np.ones((480, 640, 3), dtype=np.uint8) * 200
        orange_hsv = np.array([[[15, 200, 200]]], dtype=np.uint8)
        orange_bgr = cv2.cvtColor(orange_hsv, cv2.COLOR_HSV2BGR)[0][0]
        cv2.rectangle(img, (200, 150), (440, 330), orange_bgr.tolist(), -1)
        cv2.rectangle(img, (200, 150), (440, 330), (50, 50, 50), 2)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'camera_{self.frame_id}'
        self.frame_id += 1
        self.camera_pub.publish(msg)

    def publish_bboxes(self):
        # Fake battery as PoseArray: 4 corners as poses
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        corners = [
            (200.0, 150.0), (440.0, 150.0),
            (440.0, 330.0), (200.0, 330.0),
        ]
        for x, y in corners:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.bbox_pub.publish(msg)

    def spin_once(self):
        now = time.time()

        self.publish_joints()

        if int(now * self.rate_camera) != int((now - 0.1) * self.rate_camera):
            self.publish_camera()

        if int(now * self.rate_bboxes) != int((now - 0.2) * self.rate_bboxes):
            self.publish_bboxes()


def main(args=None):
    rclpy.init(args=args)
    node = MockDataNode()

    rate_j = 1.0 / node.rate_joints
    try:
        while rclpy.ok():
            node.spin_once()
            time.sleep(rate_j)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
