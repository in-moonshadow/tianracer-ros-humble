#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 开放地图 TEB 竞速链路（ROS 2 版）：
#   Nav2 导航（替代 ROS 1 move_base + teb_local_planner）
#   + nav_sim.py (cmd_vel -> /drive, 供仿真器使用)
#   + run3.py (经 Nav2 /navigate_to_pose 依次驶向固定路点)
# 注意：如需 TEB carlike 控制器，需安装 teb_local_planner(ros2) 并改用
#       tianracer_navigation2 的 navfn_teb 参数文件。

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_launch_dir = get_package_share_directory('tianracer_navigation2')
    default_map = os.path.join(
        get_package_share_directory('tianracer_slam'), 'maps', 'levine.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to map yaml file to load'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Whether to start RViz'),
        DeclareLaunchArgument('speed_param', default_value='3.5',
                              description='直线行驶速度 (m/s)'),
        DeclareLaunchArgument('P_param', default_value='0.0',
                              description='转弯加速比例'),

        # Nav2 导航栈
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'launch', 'nav2.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_sim_time': 'true',
            }.items(),
        ),

        # cmd_vel -> /drive (仿真器 AckermannDriveStamped)
        Node(
            package='tianracer_competition',
            executable='nav_sim.py',
            name='nav_sim',
            output='screen',
        ),

        # 固定路点导航
        Node(
            package='tianracer_competition',
            executable='run3.py',
            name='move_test',
            output='screen',
            parameters=[{
                'speed_param': LaunchConfiguration('speed_param'),
                'P_param': LaunchConfiguration('P_param'),
            }],
        ),
    ])
