#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# created by Wu GengQian (ROS 1)
# ROS 2 Humble 移植：disparity extender 开放地图竞速算法

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

FILTER_VALUE = 10.0
DISPARITY_DIF = 0.6
CAR_WIDTH = 0.16


class DisparityExtender(Node):
    def __init__(self):
        super().__init__('disparity_extender')
        self.declare_parameter('speed_param', 3.5)
        self.declare_parameter('P_param', 0.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')

        self.speed_param = self.get_parameter('speed_param').value
        self.p_param = self.get_parameter('P_param').value

        # 里程计估计的当前位姿，仿真环境下很准确
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        self.scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.disparity_extender_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('drive_topic').value, 1)

    def get_range(self, data, angle, deg=True):
        if deg:
            angle = np.deg2rad(angle)
        dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
        if dis < data.range_min or dis > data.range_max:
            dis = FILTER_VALUE
        return dis

    def disparity_extender_callback(self, data):
        dis = []
        for angle in range(-90, 91):
            dis.append(self.get_range(data, angle))

        disparities = []
        for i in range(len(dis)):
            if i == len(dis) - 1:
                continue
            if abs(dis[i] - dis[i + 1]) > DISPARITY_DIF:
                min_dis = min(dis[i], dis[i + 1])
                angle_range = math.ceil(
                    math.degrees(math.atan(CAR_WIDTH / 2 / min_dis)))
                angle_range += 15
                side_range = range(int(i - angle_range + 1), i + 1) if dis[i + 1] == min_dis else range(i + 1, int(i + 1 + angle_range))
                disparities.append((min_dis, side_range))

        for min_dis, side_range in disparities:
            for i in side_range:
                if i >= 0 and i < len(dis):
                    dis[i] = min(dis[i], min_dis)

        # 对开放地图特定位置的激光数据进行裁剪，以下适用于 levine 这张地图
        if self.pose_x < -12 and self.pose_y > 7:
            for i in range(0, 90):
                dis[i] = 0
        elif self.pose_x < -13.5 and self.pose_y < 1.5:
            for i in range(0, 120):
                dis[i] = 0
        elif self.pose_x > 8 and self.pose_y < 1:
            for i in range(0, 90):
                dis[i] = 0

        max_index = np.argmax(dis)
        max_dis = dis[max_index]

        angle = max_index - 90 if abs(max_index - 90) > 15 else 0
        angle = angle * np.pi / 180
        speed = self.speed_param + self.p_param * abs(angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        _, _, self.pose_yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtender()
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
