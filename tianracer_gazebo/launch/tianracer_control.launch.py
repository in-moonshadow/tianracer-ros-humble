#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# gz-sim 控制链：controller_manager 与控制器由 gz_ros2_control 插件
# （URDF 中配置 tianracer_control.yaml）自动加载，本 launch 只启动
# ackermann -> 车轮/转向命令节点 与 cmd_vel -> ackermann 转换。

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# gz_ros2_control 插件创建 controller_manager 节点后，需用 spawner 加载控制器
CONTROLLERS = ['joint_state_broadcaster', 'wheel_velocity_controller', 'steering_position_controller']

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace == '/' else default_namespace


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=default_namespace,
                              description='Top-level namespace'),
    ] + [
        # 加载控制器（controller_manager 由 gz_ros2_control 插件在模型创建时建立）。
        # 单个 spawner 进程加载全部控制器：省掉 2 次进程启动 + DDS 发现开销。
        # 不再固定延时空等：--controller-manager-timeout 会在服务出现前自行等待。
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=CONTROLLERS + ['-c', 'controller_manager',
                                    '--controller-manager-timeout', '30',
                                    '--activate-as-group'],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),
    ] + [
        # ackermann_cmd_stamped -> 车轮速度/转向命令（ros2_control 组控制器）
        Node(
            package='tianracer_gazebo',
            executable='servo_commands.py',
            name='servo_commands',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),

        # cmd_vel (Twist) -> ackermann_cmd_stamped（仿真 nav_sim）
        Node(
            package='tianracer_gazebo',
            executable='transform.py',
            name='nav_sim',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),
    ])
