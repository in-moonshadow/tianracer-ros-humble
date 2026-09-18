#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 裁判系统 launch：只启动 judge_system 裁判节点与计分板窗口。
# 主程序（f1tenth_racer 竞速状态机）不在此启动——
# 由裁判在点「启动」时自己 Popen 拉起，重置/完赛/犯规/退出时终止。
# 用法：
#   ros2 launch tianracer_gazebo judge.launch.py
# 需先由 tianracer_on_racetrack.launch.py 拉起 gz-sim + 控制链 + Nav2。

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace == '/' else default_namespace

# 出生位姿单一来源：tianracer_gazebo/scripts/track_spawn.py
# （与 gz spawn、AMCL 初始位姿共用同一张表）。
_GZ_LIB_DIR = os.path.abspath(os.path.join(
    get_package_share_directory('tianracer_gazebo'), '..', '..', 'lib', 'tianracer_gazebo'))
if _GZ_LIB_DIR not in sys.path:
    sys.path.insert(0, _GZ_LIB_DIR)
from track_spawn import DEFAULT_TRACK, get_spawn  # noqa: E402

# 本 launch 的默认赛道：优先环境变量（与 judge_system.py:112 的读取方式一致），
# 否则用 track_spawn 的默认赛道。
_DEFAULT_WORLD = os.environ.get("TIANRACER_WORLD") or DEFAULT_TRACK
# reset 时把车送回这里，并把同位姿重发给 AMCL —— 必须与 gz 里的实际生成点一致，
# 否则车会被送错位置、AMCL 也随之定错位。故从同一张表取，不再各写一份硬编码。
_INIT_X, _INIT_Y, _INIT_Z, _INIT_YAW = get_spawn(_DEFAULT_WORLD)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=default_namespace,
                              description='Top-level namespace'),
        DeclareLaunchArgument('world', default_value=_DEFAULT_WORLD,
                              description='赛道 world（决定 check_points.yaml）'),
        DeclareLaunchArgument('lap_count', default_value='3',
                              description='需完成圈数（每圈 3 门）'),
        DeclareLaunchArgument('speed_score_max', default_value='35.0',
                              description='速度分上限（原版模块全局 speed_score_total 初值）'),
        DeclareLaunchArgument('score_alpha', default_value='14.0',
                              description='原版模块全局 alpha；满速度分用时 = 3 * alpha = 42s'),
        DeclareLaunchArgument('stop_time_threshold', default_value='10.0',
                              description='停车判犯规阈值（秒，官方规则：超 10s 未运动即停止计时）'),
        DeclareLaunchArgument('checkpoint_timeout', default_value='30.0',
                              description='过门超时终止阈值（秒，官方规则：距上次标记点检测超 30s 即终止）。'
                                          '注意当前车速下单门间隔 18~51s，提速前会切掉正常比赛，调试期可调大'),
        # 出生位姿：默认值来自 track_spawn（与 gz spawn、AMCL 初始位姿同源）。
        # 若要临时覆盖，显式传 init_x:=... 即可；但正常换赛道只需 world:= 正确。
        DeclareLaunchArgument('init_x', default_value=str(_INIT_X),
                              description='起点 x（reset 时用；默认取自 track_spawn）'),
        DeclareLaunchArgument('init_y', default_value=str(_INIT_Y),
                              description='起点 y（reset 时用；默认取自 track_spawn）'),
        DeclareLaunchArgument('init_z', default_value=str(_INIT_Z),
                              description='起点 z（reset 时用；默认取自 track_spawn）'),
        DeclareLaunchArgument('init_yaw', default_value=str(_INIT_YAW),
                              description='起点 yaw（reset 时用；默认取自 track_spawn）'),
        DeclareLaunchArgument('enable_display', default_value='true',
                              description='是否启动计分板窗口（tkinter，需显示环境）'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='使用仿真时钟（与 Nav2 对齐，避免 judge/racer 时钟源不一致）'),

        # 裁判节点
        Node(
            package='tianracer_gazebo',
            executable='judge_system_node.py',
            name='judge_system',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world': LaunchConfiguration('world'),
                'lap_count': LaunchConfiguration('lap_count'),
                'speed_score_max': LaunchConfiguration('speed_score_max'),
                'score_alpha': LaunchConfiguration('score_alpha'),
                'stop_time_threshold': LaunchConfiguration('stop_time_threshold'),
                'checkpoint_timeout': LaunchConfiguration('checkpoint_timeout'),
                'init_x': LaunchConfiguration('init_x'),
                'init_y': LaunchConfiguration('init_y'),
                'init_z': LaunchConfiguration('init_z'),
                'init_yaw': LaunchConfiguration('init_yaw'),
            }],
            output='screen',
        ),
        # 计分板窗口：订阅 /score_display 显示实时分数（tkinter 实现）。
        # 无显示环境时节点自身会告警并跳过，不影响计分。
        Node(
            package='tianracer_gazebo',
            executable='judge_display.py',
            name='judge_display',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'topic': 'score_display',
                'world': LaunchConfiguration('world'),
                'robot_name': LaunchConfiguration('namespace'),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_display')),
        ),
    ])
