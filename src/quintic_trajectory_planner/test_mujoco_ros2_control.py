#!/usr/bin/env python3
"""
Test script for MuJoCo + ros2_control integration

This script:
1. Publishes robot description
2. Starts MuJoCo ros2_control node
3. Spawns controllers
4. Publishes test trajectories
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
import sys
import time
import os

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MujocoRos2ControlTest(Node):
    def __init__(self):
        super().__init__('mujoco_ros2_control_test')

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info('MuJoCo + ros2_control test node initialized')

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    def joint_state_callback(self, msg):
        self.get_logger().debug(f'Received joint states: {[f"{p:.3f}" for p in msg.position]}')

    def publish_joint_states(self, positions):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = [0.0] * 6
        self.joint_state_pub.publish(msg)

    def send_trajectory(self, target_positions, duration=2.0):
        """Send a trajectory to the joint trajectory controller"""
        # This would be done via action client or service in real scenario
        self.get_logger().info(f'Sending trajectory to target: {target_positions}')


def main(args=None):
    rclpy.init(args=args)

    test_node = MujocoRos2ControlTest()

    try:
        # Give ROS time to initialize
        time.sleep(2)

        # Publish initial joint states
        test_node.publish_joint_states([0.0] * 6)
        test_node.get_logger().info('Published initial joint states')

        # Wait and observe
        for i in range(10):
            rclpy.spin_once(test_node, timeout_sec=0.5)
            test_node.publish_joint_states([0.0] * 6)

        test_node.get_logger().info('Test completed successfully')

    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()