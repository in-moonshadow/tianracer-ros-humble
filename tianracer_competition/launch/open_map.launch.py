#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 开放地图竞速：disparity extender，带里程计位置相关的激光裁剪。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('speed_param', default_value='3.5',
                              description='直线行驶速度 (m/s)'),
        DeclareLaunchArgument('P_param', default_value='0.0',
                              description='转弯加速比例'),
        Node(
            package='tianracer_competition',
            executable='run_in_open_map.py',
            name='disparity_extender',
            output='screen',
            parameters=[{
                'speed_param': LaunchConfiguration('speed_param'),
                'P_param': LaunchConfiguration('P_param'),
            }],
        ),
    ])
