#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_competition) -> ROS 2 Humble 移植：
# 通过 Nav2 /navigate_to_pose action 依次驶向 levine 地图固定路点。

import threading
import time

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry


class MoveTest(Node):
    def __init__(self):
        super().__init__('move_test')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('navigate_action', '/navigate_to_pose')

        self.x = 0.0
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_callback, 1)

        self.action_client = ActionClient(
            self, NavigateToPose, self.get_parameter('navigate_action').value)

        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available')
        else:
            self.get_logger().info('Connected to Nav2 navigate_to_pose server')

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x

    def send_goal(self, x, y, qz, qw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.get_logger().info('Going to: x=%.2f y=%.2f' % (x, y))
        self.action_client.send_goal_async(goal)

    def wait_until(self, pred, timeout=30.0):
        deadline = time.time() + timeout
        while not pred() and time.time() < deadline:
            time.sleep(0.1)

    def move1(self):
        # target (7, 0)
        self.send_goal(7.0, 0.0, -0.0015, 0.99)
        self.wait_until(lambda: self.x > 3.5)
        self.move2()

    def move2(self):
        # target (-7, 8.7)
        self.send_goal(-7.0, 8.7, 0.99, 0.0016)
        self.wait_until(lambda: self.x > -4.5)
        self.move3()

    def move3(self):
        # target (-7, 0)
        self.send_goal(-7.0, 0.0, -0.0015, 0.99)
        self.wait_until(lambda: self.x > -12.2)
        self.move1()


def main(args=None):
    rclpy.init(args=args)
    node = MoveTest()
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        for _ in range(100):
            node.move1()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
