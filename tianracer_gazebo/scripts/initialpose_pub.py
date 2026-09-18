#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_gazebo) -> ROS 2 Humble 移植：
# 向 Nav2 AMCL 发布初始位姿（initialpose）。

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameter('topic', 'initialpose')
        self.declare_parameter('x_pos', 0.0)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('z_pos', 0.0)
        self.declare_parameter('R_pos', 0.0)
        self.declare_parameter('P_pos', 0.0)
        self.declare_parameter('Y_pos', 0.0)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('wait_timeout', 60.0)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('topic').value, 10)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.remaining = self.get_parameter('publish_count').value
        self._wait_timeout = self.get_parameter('wait_timeout').value
        self._waited = 0.0

    def publish_once(self):
        if self.remaining <= 0:
            self.destroy_timer(self.timer)
            return
        # 等 AMCL 订阅后再发：Nav2 可能比本节点晚启动，
        # 早发的 initialpose 会因无订阅者被丢弃。超时后仍发布，避免无订阅者时永久阻塞。
        if self.publisher.get_subscription_count() == 0:
            self._waited += 0.5
            if self._waited < self._wait_timeout:
                if int(self._waited * 2) % 10 == 1:
                    self.get_logger().info('等待 initialpose 订阅者（AMCL）...')
                return
            self.get_logger().warn('等待 initialpose 订阅者超时，仍发布')
        self.remaining -= 1

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.get_parameter('x_pos').value
        msg.pose.pose.position.y = self.get_parameter('y_pos').value
        msg.pose.pose.position.z = self.get_parameter('z_pos').value
        quat = quaternion_from_euler(
            self.get_parameter('R_pos').value,
            self.get_parameter('P_pos').value,
            self.get_parameter('Y_pos').value)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        for i in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[i] = 0.1
        self.publisher.publish(msg)
        self.get_logger().info('Published initialpose (x=%.2f y=%.2f yaw=%.2f)'
                               % (msg.pose.pose.position.x, msg.pose.pose.position.y,
                                  self.get_parameter('Y_pos').value))


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
