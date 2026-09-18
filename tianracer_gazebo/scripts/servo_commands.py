#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_gazebo) -> ROS 2 Humble 移植：
# ackermann_cmd_stamped -> 车轮速度 / 转向命令（ros2_control 组控制器）。

import math

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped


class ServoCommands(Node):
    def __init__(self):
        super().__init__('servo_commands')
        self.declare_parameter('cmd_topic', 'ackermann_cmd_stamped')
        self.declare_parameter('wheel_controller', 'wheel_velocity_controller/commands')
        self.declare_parameter('steer_controller', 'steering_position_controller/commands')
        # 阿克曼几何与传动比。取值来自本仿真链实际使用的模型
        # tianracer_gazebo/urdf/tianracer.xacro（前轴 x=+0.13、后轴 x=-0.13、轮距 0.13）：
        #   轴距 0.26m（原为 0.265，该值不在任何模型中）；
        #   轮半径 0.032m、轮距 0.13m 与模型一致。
        # ⚠️ 另有 tianracer_description/urdf/tianracer_compact.urdf（轴距 0.261），
        #    那是 TF/RViz 与真机链用的模型，不要混用。
        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('wheelbase', 0.26)
        self.declare_parameter('track_width', 0.13)

        self.emergency_brake_active = False

        # 车轮速度（4）：[left_rear, right_rear, left_front, right_front]
        self.pub_wheel = self.create_publisher(
            Float64MultiArray, self.get_parameter('wheel_controller').value, 1)
        # 转向位置（2）：[left_steering, right_steering]
        self.pub_steer = self.create_publisher(
            Float64MultiArray, self.get_parameter('steer_controller').value, 1)

        self.sub = self.create_subscription(
            AckermannDriveStamped, self.get_parameter('cmd_topic').value,
            self.set_throttle_steer, 1)
        self.srv = self.create_service(Empty, 'emergency_brake', self.emergency_brake)

        self.get_logger().info('servo_commands started')

    def set_throttle_steer(self, data):
        if self.emergency_brake_active:
            wheel = Float64MultiArray()
            wheel.data = [0.0, 0.0, 0.0, 0.0]
            steer = Float64MultiArray()
            steer.data = [0.0, 0.0]
            self.pub_wheel.publish(wheel)
            self.pub_steer.publish(steer)
            return

        wheel_radius = self.get_parameter('wheel_radius').value
        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value

        # v = ω·r -> ω = v/r（轮半径 0.032 m 时 1 m/s 对应 31.25 rad/s）
        throttle = data.drive.speed / wheel_radius
        steering_angle = data.drive.steering_angle

        # 阿克曼转向几何
        tan_steer = math.tan(steering_angle)
        left_steer = math.atan2(wheelbase * tan_steer, wheelbase - track_width * tan_steer / 2)
        right_steer = math.atan2(wheelbase * tan_steer, wheelbase + track_width * tan_steer / 2)

        wheel = Float64MultiArray()
        wheel.data = [throttle, throttle, throttle, throttle]
        steer = Float64MultiArray()
        steer.data = [left_steer, right_steer]
        self.pub_wheel.publish(wheel)
        self.pub_steer.publish(steer)

    def emergency_brake(self, request, response):
        self.emergency_brake_active = not self.emergency_brake_active
        state = 'deactivated' if not self.emergency_brake_active else 'activated'
        self.get_logger().warn('Emergency brake %s! Call the service again to toggle.' % state)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServoCommands()
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
