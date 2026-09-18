#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 赛道检查门可视化 launch：把 3 个检查门画到 RViz 并实时高亮「当前待通过」的门。
# 门几何与裁判完全同源（同一 check_points.yaml + 同一两两配对规则），
# 高亮状态来自裁判发布的 /score_display 的 points 字段。
#
# 用法：
#   ros2 launch tianracer_gazebo door_markers.launch.py
#   可选：world:=<赛道名>（默认取 TIANRACER_WORLD）| ns:=<命名空间>
#
# 注：一键竞速 launch（tianracer_race.launch.py）已内含本节点，无需单独启动。

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = "" if default_namespace in ("", "/") else default_namespace


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.environ.get("TIANRACER_WORLD", "tianracer_racetrack"),
            description='赛道名（决定读哪个 <world>_check_points.yaml）'),
        DeclareLaunchArgument(
            'namespace', default_value=default_namespace,
            description='顶层命名空间（与裁判对齐；决定 score_display 话题）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（与 Nav2/裁判对齐）'),

        Node(
            package='tianracer_gazebo',
            executable='door_markers.py',
            name='door_markers',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world': LaunchConfiguration('world'),
            }],
            output='screen',
        ),
    ])
