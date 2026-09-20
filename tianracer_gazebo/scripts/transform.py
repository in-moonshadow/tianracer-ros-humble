#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_gazebo) -> ROS 2 Humble 移植：
# cmd_vel (Twist) -> ackermann_cmd_stamped（仿真用 nav_sim）。

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class NavSim(Node):
    def __init__(self):
        super().__init__('nav_sim')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('drive_topic', 'ackermann_cmd_stamped')
        self.declare_parameter('frame_id', 'base_footprint')
        # 轴距：取自本仿真链实际使用的机器人模型 tianracer_gazebo/urdf/tianracer.xacro
        # （前轴 steering_hinge_joint x=+0.13、后轴 rear_wheel_joint x=-0.13 ⇒ 0.26m）。
        #
        # ⚠️ 本仓库存在**两份几何不同的机器人模型**，勿混用：
        #   - 仿真链（本文件/servo_commands 驱动）：tianracer_gazebo/urdf/tianracer.xacro
        #       → 前轮 x=+0.13、后轮 x=-0.13，轴距 **0.26**
        #   - TF/RViz 用：tianracer_description/urdf/tianracer_compact.urdf
        #       → 前轮 x=+0.261、后轮在 base_link 原点，轴距 0.261
        #   vehicle_geometry.yaml（0.261）服务的是**真机链**（bringup/navigation/teleop），
        #   不要拿它来驱动本仿真转换器。
        self.declare_parameter('wheelbase', 0.26)
        # 转向角上限：URDF steering_hinge_joint 的 limit 为 ±0.6 rad
        self.declare_parameter('max_steering_angle', 0.6)

        self.pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('drive_topic').value, 1)
        self.sub = self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self.callback, 1)

    def _steering_angle(self, v, omega):
        """cmd_vel -> 转向角。

        对齐 ROS1 tianracer_navigation/script/cmd_vel_to_ackermann_drive.py 的
        convert_trans_rot_vel_to_steering_angle（几何正确式）：δ = atan(L·ω/v)。

        本文件原先沿用 ROS1 仿真版 transform.py 的写法 steering_angle = angular.z，
        把角速度(rad/s)当角度(rad)用：规划 ω=0.5 时，vx=1.0 实际转成 2.06rad/s（过转
        4 倍）、vx=0.2 只有 0.41rad/s（欠转），执行转向量与规划值不成比例，车必然
        越走越偏，最终触发 No valid trajectories。原项目本就有正确实现在上面那个文件里。
        """
        if omega == 0.0 or abs(v) < 1e-3:
            return 0.0
        steer = math.atan(self.get_parameter('wheelbase').value * omega / v)
        limit = self.get_parameter('max_steering_angle').value
        return max(-limit, min(limit, steer))

    def callback(self, data):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.drive.speed = data.linear.x
        msg.drive.acceleration = 1.0
        msg.drive.jerk = 1.0
        msg.drive.steering_angle = self._steering_angle(data.linear.x, data.angular.z)
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
