#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_competition) -> ROS 2 Humble 移植：cmd_vel -> /drive (AckermannDriveStamped)
#
# 2026-09-13 修复两处移植遗留问题（此前本节点实际无法工作）：
#   1. 订阅曾带 raw=True，回调收到的是 bytes 而非 Twist，一收到消息即 AttributeError；
#   2. angular.z（偏航角速度）被直接当作舵角赋值，缺少 δ=atan(L·ω/v) 换算。
# 另注：/drive 目前无订阅者（ROS 1 时期即如此，属 F1TENTH 上游遗留），本节点保留话题拓扑。

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class NavSim(Node):
    def __init__(self):
        super().__init__('nav_sim')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('wheelbase', 0.261)

        self.wheelbase = self.get_parameter('wheelbase').value

        self.pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('drive_topic').value, 1)
        self.sub = self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self.callback, 1)

    def callback(self, data):
        speed = data.linear.x
        # angular.z 是偏航角速度(rad/s)，不是舵角(rad)：须做几何换算 δ=atan(L·ω/v)。
        # 本文件此前沿用 ROS1 旧写法直接赋值，会过转/欠转数倍；ROS2 版
        # tianracer_gazebo/scripts/transform.py:31-45 已修正，此处对齐它。
        # v 趋零时用 v_eff 兜底，避免除零。
        v_eff = speed if abs(speed) > 1e-3 else 0.1
        steering_angle = math.atan(self.wheelbase * data.angular.z / v_eff)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.drive.speed = speed
        msg.drive.acceleration = 1.0
        msg.drive.jerk = 1.0
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 1.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavSim()
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
