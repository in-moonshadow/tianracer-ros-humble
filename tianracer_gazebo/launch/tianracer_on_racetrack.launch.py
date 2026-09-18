#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 在 gz-sim (Fortress) 世界内生成 tianracer 并启动控制链（ROS 2 + ros_gz 版）。
# 传感器数据经 ros_gz_bridge 桥接；里程计由 gz odometry 系统插件桥接。

import math
import os
import subprocess
import sys
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace == '/' else default_namespace

# 出生位姿的单一来源：scripts/track_spawn.py（与 AMCL 参数、裁判 reset 共用同一张表）。
# 该模块装在 lib/tianracer_gazebo/，launch 执行时不在 sys.path 上，故显式注入。
_GZ_LIB_DIR = os.path.abspath(os.path.join(
    get_package_share_directory('tianracer_gazebo'), '..', '..', 'lib', 'tianracer_gazebo'))
if _GZ_LIB_DIR not in sys.path:
    sys.path.insert(0, _GZ_LIB_DIR)
from track_spawn import DEFAULT_TRACK, get_spawn  # noqa: E402


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('tianracer_gazebo')
    xacro_file = os.path.join(gazebo_pkg, 'urdf', 'tianracer_run.xacro')

    # 直接启动 gz sim 服务端。
    # 必须用 ign gazebo --force-version 6（v6 才能加载 gz_ros2_control 插件；
    # gz sim 默认 v7 会报 "does not export any plugins"）。
    # 重要：不要设置 NVIDIA EGL 环境变量（__GLX_VENDOR_LIBRARY_NAME 等）——实测会导致
    # v6 服务端挂起（create 服务无响应）。gpu_lidar 无需显式 NVIDIA env 也能渲染。
    # gui:=false 时加 -s（仅服务端）；不要用 --headless-rendering。
    def start_gz_sim(context):
        gui = context.launch_configurations.get('gui', 'true')
        world = context.launch_configurations.get('world', 'tianracer_racetrack.world')
        world_path = os.path.join(gazebo_pkg, 'worlds', world)
        cmd = ['ign', 'gazebo', '--force-version', '6', '-r']
        if gui == 'false':
            cmd.append('-s')
        cmd.append(world_path)
        # 用 bash -c 复刻直接测试的调用方式（ExecuteProcess 列表式调用偶发 gz 服务挂起）
        return [ExecuteProcess(cmd=['bash', '-c', ' '.join(cmd)], output='screen')]

    # 让 gz-sim SystemLoader 能找到 gz_ros2_control 与 odometry 系统插件。
    # 从环境动态推导（不写死发行版路径）：优先取 GZ_SIM_SYSTEM_PLUGIN_PATH（若外部已设），
    # 否则用 ros2 pkg prefix + pkg-config 拼，最后回退 Humble 默认位置。
    def _default_gz_plugin_path():
        if os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'):
            return os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']
        try:
            from ament_index_python.packages import get_package_prefix
            ros_lib = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
        except Exception:
            ros_lib = '/opt/ros/humble/lib'
        gz_plugindir = None
        try:
            import subprocess
            out = subprocess.run(
                ['pkg-config', '--variable=libdir', 'ignition-gazebo6'],
                capture_output=True, text=True, check=True).stdout.strip()
            candidate = os.path.join(out, 'ign-gazebo-6', 'plugins')
            if os.path.isdir(candidate):
                gz_plugindir = candidate
        except Exception:
            pass
        if not gz_plugindir:
            gz_plugindir = '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins'
        return os.pathsep.join([ros_lib, gz_plugindir])

    gz_plugin_path = _default_gz_plugin_path()

    # gz-sim 的资源搜索路径（`model://` 解析）。
    # ⚠️ raicom.world 与 race_with_cones.world 引用 `model://construction_cone/meshes/construction_cone.dae`，
    # 而该模型只存在于 **Gazebo 11 的模型库**（/usr/share/gazebo-11/models，随 gazebo11 包安装），
    # 不在 gz-sim7 的默认搜索路径里 → 实测报
    #   [Err] [Ogre2MeshFactory.cc:125] Failed to get Ogre item for [model://construction_cone/...]
    # 后果是锥体的 collision 取不到网格，锥体落在车前方 1.3m 处把车**物理卡死**
    # （症状：cmd_vx 满速而 odom_vx 恒 0）。故必须把 Gazebo 11 模型目录并入资源路径。
    def _gz_resource_path():
        pkg_share = get_package_share_directory('tianracer_gazebo')
        paths = [os.path.join(pkg_share, 'worlds')]
        # 已存在的环境值一并保留（外部可能已设）
        for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
            for p in os.environ.get(var, '').split(os.pathsep):
                if p and p not in paths:
                    paths.append(p)
        # Gazebo 11 模型库（first-found-wins，追加在末尾不影响本仓库自带模型）
        for cand in ('/usr/share/gazebo-11/models', os.path.expanduser('~/.gazebo/models')):
            if os.path.isdir(cand) and cand not in paths:
                paths.append(cand)
        return os.pathsep.join(paths)

    gz_resource_path = _gz_resource_path()

    # xacro 生成的 robot_description（rsp 与 create 共用）
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file, ' "prefix:=\'',
                 LaunchConfiguration('namespace'), '\'"']),
        value_type=str)

    # 出生位姿的唯一来源：track_spawn.TRACK_SPAWNS。此处不写任何位姿字面量——
    # 同一张表也被 AMCL 参数（nav2.launch.py 注入）与裁判 reset（judge.launch.py）使用。
    def resolve_spawn_pose(context):
        world = context.launch_configurations.get('world') or DEFAULT_TRACK
        return get_spawn(world)

    # 生成机器人 URDF 文件并用 gz service create（sdf_filename 方式）生成模型。
    # 传感器只有在文件方式生成时才被 gz-sim Sensors 系统正确挂载（实测）。
    def spawn_robot(context):
        ns = context.launch_configurations.get('namespace', default_namespace)
        x, y, z, yaw = resolve_spawn_pose(context)
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        urdf_cmd = ExecuteProcess(
            cmd=['bash', '-c',
                 f"xacro {xacro_file} \"prefix:='{ns}'\" > /tmp/tianracer_robot.urdf"],
            output='screen')

        req = ("name: 'tianracer', "
               f"sdf_filename: '/tmp/tianracer_robot.urdf', "
               f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
               f"orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}}}")
        # 必须用 v6 的 ign service 客户端调用 create：v7 的 gz service 客户端与 v6
        # 服务端存在传输版本不匹配，create 服务响应偶发丢失导致超时（实测 ign service 稳定返回 data:true）。
        create_cmd = ExecuteProcess(
            cmd=['ign', 'service', '-s', '/world/default/create',
                 '--reqtype', 'ignition.msgs.EntityFactory',
                 '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '20000',
                 '--req', req],
            output='screen')

        # URDF 若已与 gz 启动并行预生成（prebuilt_urdf:=true），
        # 直接 create，省掉 xacro 串行等待（实测约 0.49s）。
        if context.launch_configurations.get('prebuilt_urdf', 'false') == 'true':
            return [create_cmd]

        # create 必须等 urdf 文件写完再执行：两个 ExecuteProcess 若同时启动，
        # ign service create 可能读到尚未写完（或上一轮残留）的 urdf。
        return [urdf_cmd,
                RegisterEventHandler(OnProcessExit(target_action=urdf_cmd,
                                                   on_exit=[create_cmd]))]

    # 等 gz 的 create 服务就绪后再 spawn。start_gz:=false（gz 已由外部启动）时已先等过一次，
    # 这里再兜住直接以 start_gz:=true 启动的路径。gz CLI 单次调用约 2.4s（传输握手固定成本），
    # 因此循环内不再额外 sleep。
    def wait_and_spawn(context):
        deadline = time.time() + 30.0
        while time.time() < deadline:
            try:
                out = subprocess.run(['gz', 'service', '-l'],
                                     capture_output=True, text=True, timeout=10).stdout
                if '/world/default/create' in out:
                    break
            except Exception:
                pass
        return spawn_robot(context)

    # start_gz:=false 时 gz 已由外部预先启动并确认 create 服务就绪，无需再探测
    #（gz CLI 单次握手约 2.4s，重复探测纯属浪费）；只有 launch 自己启 gz 时才需等待。
    def spawn_when_ready(context):
        if context.launch_configurations.get('start_gz', 'true') == 'false':
            return spawn_robot(context)
        return wait_and_spawn(context)

    # 启动即自动定位已改由 AMCL 自带参数承担（见 tianracer_navigation2/params/navfn_dwb_nav2_params.yaml
    # 的 set_initial_pose / initial_pose.*）——在 configure 阶段完成，比发 initialpose 话题更快更稳。
    # initialpose_pub.py 仍保留，供需要手动改定位时使用。

    return LaunchDescription([
        # 仅设置 gz 系统插件路径（gz_ros2_control/odometry 插件）。
        # 注意：不要设置 NVIDIA EGL 环境变量，否则 ign gazebo v6 服务端会挂起。
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plugin_path),
        # 资源搜索路径：含本包 worlds（赛道 mesh/纹理）+ Gazebo 11 模型库（raicom 的 construction_cone）。
        # 两个变量名都设：gz-sim6/7 分别读 IGN_GAZEBO_RESOURCE_PATH / GZ_SIM_RESOURCE_PATH。
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),

        DeclareLaunchArgument('namespace', default_value=default_namespace,
                              description='Top-level namespace'),
        DeclareLaunchArgument('start_gz', default_value='true',
                              description='Whether to start the gz-sim server (set false if it was already started externally)'),
        DeclareLaunchArgument('world', default_value='tianracer_racetrack.world',
                              description='World file name in worlds/ (e.g. tianracer_racetrack.world)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Whether to start gz-sim GUI'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('prebuilt_urdf', default_value='false',
                              description='URDF 已预生成（跳过 launch 内 xacro）'),

        # gz-sim 服务端。可设 start_gz:=false 由外部先直接启动 gz——
        # 实测 launch 内启动的 gz server 偶发 100% CPU 空转、create 服务无响应。
        OpaqueFunction(
            function=start_gz_sim,
            condition=IfCondition(LaunchConfiguration('start_gz'))),

        # robot_state_publisher：发布 robot_description（gz_ros2_control 读取）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content,
            }],
            output='screen',
        ),

        # 在 gz-sim 中生成机器人。
        # 关键：必须用 gz service create + sdf_filename 文件方式生成，传感器才能正确挂载
        #（ros_gz_sim create 节点用字符串生成模型时传感器不更新，实测无数据）。
        # 原为 period=20.0 固定死等 gz 就绪；现改为就绪即 spawn。
        # start_gz:=false（gz 已由外部启动）下直接 spawn，不再重复探测。
        TimerAction(
            period=0.1,
            actions=[OpaqueFunction(function=spawn_when_ready)],
        ),

        # ros_gz_bridge：gz 传感器/里程计 -> ROS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                'scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                'imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                'model/tianracer/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # gz OdometryPublisher 把 odom->base_footprint TF 发布为 gz.msgs.Pose_V
                #（话题 /model/tianracer/pose），桥接成 ROS /tf 供 slam_toolbox/Nav2 使用
                'model/tianracer/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                'camera@sensor_msgs/msg/Image@gz.msgs.Image',
                'camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                # 关键：必须桥接时钟，否则全部 use_sim_time=True 节点的时间冻结在 0
                #（裁判段用时恒 0、速度分恒 0、主程序定时器不触发、停车判罚永不生效）。
                # gz-sim Fortress 的时钟实际发布在 /world/<world>/clock；gz 侧那个
                # 同名 /clock 话题实测没有任何消息（ign topic -e -t /clock 与
                # ros2 topic hz /clock 均收不到），桥它等于没桥。
                # 故桥 /world/default/clock，再在 ROS 侧重映射回标准的 /clock。
                'world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                # gz 设定模型位姿服务桥接成 ROS 服务：裁判 reset 靠它把车送回出生点
                #（等价 ROS1 的 rosservice call /gazebo/reset_world）。
                # 为什么不用 /world/default/control 的 reset：实测 gz-sim 6.18 下
                # ControlWorld 的 reset（model_only 与 all 都试过）返回 success 却不会改动
                # 运行时 create 出来的模型位姿，且 all 还会把仿真时间清零；
                # /world/default/set_pose 能精确设定位姿且不动仿真时间。
                # 服务名必须写全路径；bridge 只支持「gz 服务暴露为 ROS 服务」这一个方向。
                '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            ],
            remappings=[('model/tianracer/odometry', 'odom'),
                        ('model/tianracer/pose', 'tf'),
                        ('world/default/clock', 'clock')],
            output='screen',
        ),

        # 控制链：spawner 加载控制器 + ackermann->车轮/转向 + cmd_vel 转换
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_pkg, 'launch', 'tianracer_control.launch.py')),
            launch_arguments=[
                ('namespace', LaunchConfiguration('namespace')),
            ],
        ),
    ])
