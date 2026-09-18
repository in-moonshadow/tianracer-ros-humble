#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Tianracer 一键竞速入口：仿真 + 控制链 + Nav2 + 裁判，启动后到计分板窗口点「启动」发车。
#
# 编排三段（与 ROS 1 原版一致的分工）：
#   ① tianracer_on_racetrack.launch.py  gz-sim + 模型生成 + bridge + 控制链
#   ② tianracer_navigation2/nav2.launch.py  Nav2 栈（必启：f1tenth_racer 走 navigate_to_pose）
#   ③ judge.launch.py  裁判 + 计分板窗口（等 Nav2 active 后才起）
#
# 用法：
#   ros2 launch tianracer_gazebo tianracer_race.launch.py
#   ros2 launch tianracer_gazebo tianracer_race.launch.py gui:=true          # 看 gz 3D 画面
#   ros2 launch tianracer_gazebo tianracer_race.launch.py use_rviz:=false    # 关 RViz
#   ros2 launch tianracer_gazebo tianracer_race.launch.py use_planner:=navfn_teb
#
# 为什么裁判要门控：f1tenth_racer.py 用 ActionClient(NavigateToPose) 走 Nav2，
# action server 未就绪时点「启动」只会得到 "navigate_to_pose action server not available"。
# 故等 /bt_navigator 为 active 后再起裁判。
#
# 主程序（f1tenth_racer）**不由本 launch 启动**：对齐 ROS 1 原版，由裁判在点「启动」时
# subprocess.Popen 拉起、在重置/完赛/犯规时终止（见 judge_system.py 的 _spawn_racer/_kill_racer）。

import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace == '/' else default_namespace

# Nav2 就绪等待上限。记忆里 Nav2 会偶发卡死（lifecycle_manager_navigation 停在
# Configuring、bt_navigator 始终 unconfigured），故必须带超时，不能无限等。
NAV2_READY_TIMEOUT = 240.0
POLL_INTERVAL = 2.0


def _nav2_active():
    """单条判据：/bt_navigator 的 lifecycle 状态为 active。

    不用 `Managed nodes are active` 计数——localization_manager 会先打印它，
    只 grep 这一条会误判全栈就绪。
    """
    try:
        out = subprocess.run(
            ['ros2', 'lifecycle', 'get', '/bt_navigator'],
            capture_output=True, text=True, timeout=15).stdout
    except Exception:
        return False
    return 'active' in out


def start_judge_when_nav2_ready(context):
    """等 Nav2 active 后再起裁判；超时则明确报错，不静默启动（否则点「启动」必然失败）。"""
    judge_launch = os.path.join(
        get_package_share_directory('tianracer_gazebo'), 'launch', 'judge.launch.py')

    # world 必须取**无后缀前缀**：judge_system 把它原样写进 TIANRACER_WORLD env
    # 交给 f1tenth_racer，后者拼 <world>_points.yaml。若传 'xxx.world' 会拼成
    # 'xxx.world_points.yaml' 而 FileNotFoundError。
    # 用 perform(context) 显式取本 launch 的 world_name —— 不能写
    # LaunchConfiguration('world')，同名会与子 launch（on_racetrack/judge）的
    # world 参数作用域冲突，被解析成带 .world 后缀的形式。
    world_prefix = context.launch_configurations.get('world_name', 'tianracer_racetrack')

    deadline = time.time() + NAV2_READY_TIMEOUT
    told = False
    while time.time() < deadline:
        if _nav2_active():
            return [LogInfo(msg='[race] Nav2 已就绪，启动裁判与计分板'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(judge_launch),
                        launch_arguments={
                            'world': world_prefix,
                            'namespace': LaunchConfiguration('namespace'),
                            'lap_count': LaunchConfiguration('lap_count'),
                            'checkpoint_timeout': LaunchConfiguration('checkpoint_timeout'),
                            'enable_display': LaunchConfiguration('enable_display'),
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                        }.items())]
        if not told:
            LogInfo(msg=f'[race] 等待 Nav2 就绪（/bt_navigator active），上限 '
                        f'{NAV2_READY_TIMEOUT:.0f}s...')
            told = True
        time.sleep(POLL_INTERVAL)

    return [LogInfo(msg='[race] ✗ Nav2 未在超时内就绪，裁判**未**启动。'
                        '请检查 Nav2 日志（常见：lifecycle_manager_navigation 卡在 Configuring）。')]


def generate_launch_description():
    gazebo_share = get_package_share_directory('tianracer_gazebo')
    nav2_share = get_package_share_directory('tianracer_navigation2')

    # world 有**两套口径**：gz 侧要文件名（xxx.world），裁判与 f1tenth_racer 要前缀（xxx，
    # 拼 <world>_points.yaml / _check_points.yaml）。用户只传前缀 world_name，内部派生。
    # ⚠️ 本参数刻意不叫 `world`：子 launch（on_racetrack / judge）都有同名 `world` 且
    # 语义不同（带后缀），同名会在 include 时作用域冲突，把前缀污染成 'xxx.world'。
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')

    map_file = LaunchConfiguration(
        'map',
        default=[gazebo_share, '/maps/', world_name, '.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='tianracer_racetrack',
                              description='赛道名前缀（无后缀，决定 world 文件与 check_points/points）'),
        DeclareLaunchArgument('world_file',
                              default_value=[world_name, '.world'],
                              description='gz 侧的 world 文件名（自动派生，一般不用改）'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='是否启动 gz-sim 3D 界面（默认关，省 CPU）'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz'),
        DeclareLaunchArgument('use_planner', default_value='navfn_dwb',
                              description='规划器参数集：navfn_dwb（默认）| navfn_teb | smac_graceful | theta_star_mppi | ...'),
        DeclareLaunchArgument('map', default_value=map_file,
                              description='地图 yaml 全路径（默认按 world 名推导）'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_display', default_value='true',
                              description='是否显示裁判计分板窗口（tkinter，需 DISPLAY）'),
        DeclareLaunchArgument('lap_count', default_value='3',
                              description='需完成圈数（每圈 3 门）'),
        DeclareLaunchArgument('checkpoint_timeout', default_value='30.0',
                              description='过门超时终止阈值（秒）'),
        DeclareLaunchArgument('namespace', default_value=default_namespace,
                              description='顶层命名空间'),

        # ① 仿真 + 控制链
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'tianracer_on_racetrack.launch.py')),
            launch_arguments={
                'start_gz': 'true',
                'world': world_file,
                'gui': LaunchConfiguration('gui'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'namespace': LaunchConfiguration('namespace'),
            }.items()),

        # ② Nav2（必启）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'nav2.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'map': LaunchConfiguration('map'),
                'use_planner': LaunchConfiguration('use_planner'),
                # world 必须转发：nav2.launch.py 用它取该赛道的出生位姿注入 amcl 的
                # initial_pose。此前漏传，world_name:=racetrack_1 时 AMCL 仍按默认赛道
                # (0,0) 初始化，而车实际生成在 16m 外（raicom 6.7m）⇒ 定位必然发散。
                # 默认赛道出生点就是 (0,0)，所以这个 bug 一直没暴露。
                'world': world_name,
            }.items()),

        # ②.5 检查门可视化（RViz 里画出 3 个门并高亮当前目标，便于对照减速位置）。
        # 不依赖 Nav2，立即启动即可；高亮状态靠订阅裁判的 /score_display，
        # 裁判起晚时先按「门0 为当前目标」渲染，收到首条 score_display 后自动校正。
        Node(
            package='tianracer_gazebo',
            executable='door_markers.py',
            name='door_markers',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world': world_name,
            }],
            output='screen',
        ),

        # ③ 裁判 + 计分板（门控在 Nav2 就绪之后）。
        # 用 TimerAction 延迟触发：OpaqueFunction 在 launch 遍历时就执行，若直接放在
        # 顶层会在 gz/Nav2 尚未启动时开始轮询，白白阻塞 launch 的进程生成。
        # 延迟 5s 起（给 gz 与 Nav2 各自的进程生成让路），随后轮询等待。
        TimerAction(
            period=5.0,
            actions=[OpaqueFunction(function=start_judge_when_nav2_ready)],
        ),
    ])
