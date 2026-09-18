#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 裁判系统：ROS 1 版本（judge_system.so + judge_system_node.py）→ ROS 2 Humble 的纯 rclpy 重写。
#
# 本文件实现：
#   - md5 模型防篡改校验（md5_check.py）
#   - 车位置监控 + 长时间未动停车检测
#   - 检查点（赛道门）通过判定（position_check.py 的 is_intersect 算法，精确还原）
#   - 评分（自定规则，见下）
#   - start / brake / reset 控制
#
# ===== 评分规则（对齐原版 judge_system.so 的 Judge.cal_racing_score）=====
#
# 赛道模型：check_points.yaml 的 N 个点两两配对成「检查门」线段（每 2 点 = 1 门，
#           通常 6 点 = 3 门）。车按顺序 0→1→2→0→1→2… 循环压门（position_check.analysis
#           内部用 ana_cnt % 3 实现），每通过 3 个门记 1 圈。
#
# 流程：
#   1. start 服务 -> md5 模型校验 -> 通过则拉起主程序（f1tenth_racer）并记录 start_time，进入计分。
#   2. 每通过一个门：按门分值表累加门分，并记录时刻。
#   3. 通过 lap_count 圈（= lap_count * 3 个门）-> 自动完赛结算，并停掉主程序。
#   4. 停车超过 stop_time_threshold 秒 -> 判犯规终止，并停掉主程序。
#   4'. 超过 checkpoint_timeout 秒未检测到任何检查门 -> 判终止，并停掉主程序
#      （车在动但绕圈出不来/定位漂移时，第 4 条「停车」判罚不会触发，靠这条兜底）。
#      阈值默认 30.0（官方规则值）；当前车速偏慢时会被触发，调试期可调大。
#   5. reset 服务 -> 对齐原版 reset_racecar 的动作序列：模型回出生位姿 -> 重发初始位姿
#      （AMCL 回起点）-> 停主程序 -> 取消导航目标 -> 刹停 -> 清代价地图 -> 重置裁判状态。
#      注意：initialpose 是等传送确认 + 沉降后再发的（异步 set_pose 无法像原版那样同步返回）。
#
# 两条终止判据均出自官方规则（docs.tianbot.com/competition/f1tenth_online/contest-rules.html
# 「电子裁判系统终止条件」附录）：「距离上一次小车运动超过 10s，会认为小车已经停止——
# 停止计时并统计分数」「距离上一次完成标记点检测时间超过 30s，认为系统已经终止——
# 停止计时并统计分数」。注意官方语义是终止并结算分数，不是扣分，移植版 _finalize 与之对齐。
#
# 主程序生命周期对齐 ROS1 原版 judge_system.so：主程序不进 launch，由裁判在「启动」时
# Popen 拉起（对应界面文案「目标代码已启动」），重置/完赛/犯规/退出时终止。故主程序自身
# 不含启动信号订阅，也不需要。
#
# 得分：总分 = 门分累计 + 速度分
#   - 门分累计：每通过一个门累加该门的固定分值，与车速无关。原版模块全局
#     score_list = [5,5,5,5,5,5,5,5,10]（9 门：前 8 门各 5 分、末门 10 分，合计 50 分），
#     与 3 圈 × 3 门逐门对应；此处按门数推广为「末门 10 分、其余每门 5 分」。
#   - 速度分：speed_score_max * min(1, 3 * score_alpha / times)，times 为总用时。
#     3 * score_alpha = 42 秒是满速度分用时，跑进 42 秒即拿满（不额外奖励更快），
#     超出后按 42/t 衰减：60s→24.5，120s→12.25，234s→6.29。
#     **只在完赛结算时并入一次**（原版语义）：比赛中与提前终止时报的都是纯门分，
#     实测原版界面在 times=90.06 时显示 25（门分），而非含速度分的 41.3。
#   - 满分 = 50 + 35 = 85。总分取 3 位小数（原版为 round，非截断）。
#   - 停车的用时代价已体现在速度分衰减里，不另设扣分项。
#
# 默认参数（可经 judge_params.yaml 或 launch 覆盖）：
#   lap_count = 3               # 需完成的圈数（每圈 3 个门）
#   speed_score_max = 35.0      # 速度分上限（= 原版模块全局 speed_score_total 初值）
#   score_alpha = 14.0          # 原版模块全局 alpha；满速度分用时 = 3 * alpha = 42s
#   stop_time_threshold = 10.0  # 秒，停车判犯规阈值（官方规则：超 10s 未运动即停止计时）
#   checkpoint_timeout = 30.0   # 秒，过门超时终止阈值（官方规则：距上次标记点检测超 30s 即终止）。
#                               # 注意：当前车速下单门间隔 18~51s，提速前会切掉正常比赛，调试期可调大

import math
import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_msgs.msg import String, Float32
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler
import tf2_ros
from tf2_ros import Buffer, TransformListener

import position_check
import md5_check


class Judger(Node):
    """比赛裁判节点：模型校验 + 停车检测 + 检查门判定 + 评分 + 控制。"""

    DOORS_PER_LAP = 3  # 每圈检查门数量（check_points.yaml 6 点两两配对 -> 3 门）
    # 门分值表（原版模块全局 score_list）：末门 10 分、其余每门 5 分。
    # 默认 3 圈 × 3 门 = 9 门 -> [5,5,5,5,5,5,5,5,10]，与原版逐元素一致，合计 50 分。
    DOOR_SCORE = 5.0
    FINAL_DOOR_SCORE = 10.0
    # gz 里的车模型名：必须与 launch 中 create 请求的 name: 'tianracer' 一致
    GZ_MODEL_NAME = 'tianracer'
    # initialpose 在传送确认后延迟这么久再发（秒）：等 gz 的新位姿经 bridge 反映到
    # odom/TF 上。发早了 AMCL 会拿传送前的 odom 去算 map->odom，定位反而偏掉。
    INITPOSE_SETTLE_SEC = 0.5
    # initialpose 的 stamp 回退量（秒）：now() 可能比最新的 odom->base_footprint 超前
    # 约一个发布周期（实测超前 10ms），AMCL 的 MessageFilter 会以「外推到未来」丢弃它。
    INITPOSE_STAMP_BACKDATE = 0.2

    def __init__(self):
        super().__init__('judge_system')

        # 命名空间（与 humble launch 一致：TIANBOT_NAME 未设置时为 ''，话题不带前缀）
        ns = os.getenv("TIANBOT_NAME", os.getenv("TIANRACER_NAME", ""))
        if ns in ("", "/"):
            ns = ""
        self._ns = ns
        self._topic = lambda name: (f"/{ns}/{name}" if ns else f"/{name}")

        # 可配置参数
        self.declare_parameter('world', os.getenv("TIANRACER_WORLD", "tianracer_racetrack"))
        self.declare_parameter('lap_count', 3)
        # 官方规则「电子裁判系统终止条件」：距离上一次小车运动超过 10s，认为小车已停止
        # ——停止计时并统计分数。（原版 .so 同样含 "(over 10s)" 的判据文案）
        self.declare_parameter('stop_time_threshold', 10.0)
        # 官方规则同上：距离上一次完成标记点检测时间超过 30s，认为系统已经终止
        # ——停止计时并统计分数。用于车仍在动但一直过不了门（绕圈/定位漂移）的兜底。
        # 默认取官方值 30.0。注意实测：本仿真实时率≈0.95（仿真时间≈墙钟），当前车速仅
        # 约 0.45m/s，单门间隔 18~51s（三圈 277.51s / 9 门，第 1 个门 50.91s），因此
        # **车提速到 0.75m/s 以上之前，这条判据会在首个门前就终止比赛**；调试期可临时
        # 调大；传 0.0 表示关闭该判据。
        self.declare_parameter('checkpoint_timeout', 30.0)
        self.declare_parameter('speed_score_max', 35.0)
        self.declare_parameter('score_alpha', 14.0)
        # 显示刷新的节流周期（秒，仿真时间）：车动驱动计时，按此周期发布分数
        self.declare_parameter('pub_interval', 0.2)
        # 出生位姿（reset 时把车送回这里并重发给 AMCL）。默认值与
        # tianracer_on_racetrack.launch.py 中 tianracer_racetrack.world 的回退位姿
        # (x_pos/y_pos/z_pos/Y_pos) 一致。
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_z', 0.1)
        self.declare_parameter('init_yaw', 1.54)

        # 状态
        self._start_flag = False
        self._stop_flag = False
        self._check_flag = False
        self._pass_point_idx = 0          # 已通过检查门数量（= position_check.ana_cnt）
        self._move_dis = 0.0              # 累计移动距离（米）
        self._nw_time = 0.0               # 当前时间（秒）
        self._start_time = 0.0            # 比赛开始时刻
        self._temp_time = 0.0             # 上一检查门通过时刻（段起点）
        self._seg_move_dis_start = 0.0    # 段起点的累计移动距离
        self._scores = 0.0                # 门分累计
        self._speed_score = 0.0           # 当前速度分
        self._total_score = 0.0           # 总分 = 门分累计 + 速度分
        self._state = 'idle'              # 对外状态：idle/running/finished/stopped（供计分板渲染）
        self._wait_check_timing = None    # 停车检测起始时刻（None=未开始，避免 sim time=0 冲突）
        self._last_position = None

        # 车动驱动计时：odom 回调按节流周期发布分数，使窗口用时随行驶连续更新。
        # 原版为 time_sub + update_time_callback；重写版只在过门时发布，导致用时不动。
        self._last_pub_time = 0.0
        self._initpose_timer = None       # reset 后延迟发 initialpose 的一次性定时器

        # 主程序（f1tenth_racer 竞速状态机）子进程句柄。
        # 对齐 ROS1 原版 judge_system.so：launch 里不含主程序，由裁判在「启动」时拉起、
        # 在 terminate_test / reset_racecar 时杀掉。原版命令为
        #   Popen("rosrun tianracer_gazebo f1tenth_racer.py __ns:=<ns>")
        # 终止为 ps aux | grep f1tenth_racer | awk '{print "kill -9", $2}' | sh
        self._racer_proc = None

        # 订阅 odom 获取车世界位置（gz odom 初始与 world 对齐，等价 ROS1 /gazebo/model_states）
        self._odom_sub = self.create_subscription(
            Odometry, self._topic('odom'), self._update_callback, 1)

        # TF 用于把车位置转到 map 系，与检查门（frame_id: map）统一坐标系。
        base_frame = f"{ns}/base_footprint" if ns else "base_footprint"
        self._base_frame = base_frame
        self._map_frame = f"{ns}/map" if ns else "map"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 评分显示
        self._score_pub = self.create_publisher(String, self._topic('score_display'), 1)
        self._move_dis_pub = self.create_publisher(Float32, self._topic('move_dis'), 1)

        # 控制服务（ROS2 服务名不支持 '~' 私有前缀，落在节点命名空间下）
        self._start_srv = self.create_service(Empty, 'start', self._start_cb)
        self._stop_srv = self.create_service(Empty, 'stop', self._stop_cb)
        self._brake_srv = self.create_service(Empty, 'brake', self._brake_cb)
        self._reset_srv = self.create_service(Empty, 'reset', self._reset_cb)

        # 刹车服务（复用 servo_commands.py 的 emergency_brake）
        self._brake_client = self.create_client(Empty, self._topic('emergency_brake'))

        # 取消导航目标：走 action 的隐藏服务 <action>/_action/cancel_goal
        #（rclpy 的 ActionClient 没有 cancel_all_goals_async，见 _cancel_nav_goals）
        self._cancel_goal_client = self.create_client(
            CancelGoal, self._topic('navigate_to_pose') + '/_action/cancel_goal')

        # initialpose 发布器（reset 用：把车送回起点后让 AMCL 跟着回到起点）
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self._topic('initialpose'), 1)

        # gz 设定模型位姿服务（等价 ROS1 的 rosservice call /gazebo/reset_world）：
        # reset 时把车放回出生点。服务名里的 world 段固定为 default——与 launch 里
        # /world/default/create、/world/default/clock 的用法一致；模型名与 launch 里
        # create 请求的 name: 'tianracer' 一致。
        self._set_pose_client = self.create_client(
            SetEntityPose, '/world/default/set_pose')

        # 清除 Nav2 代价地图（等价 ROS1 的 <ns>/move_base/clear_costmaps：
        # move_base 一个服务清全局+局部，Nav2 拆成两个服务，故两个都要调）
        self._clear_costmap_clients = [
            self.create_client(
                ClearEntireCostmap,
                self._topic('global_costmap/clear_entirely_global_costmap')),
            self.create_client(
                ClearEntireCostmap,
                self._topic('local_costmap/clear_entirely_local_costmap')),
        ]

        self._total_doors = self.get_parameter('lap_count').value * self.DOORS_PER_LAP
        self._door_scores = ([self.DOOR_SCORE] * (self._total_doors - 1)
                             + [self.FINAL_DOOR_SCORE])
        self._load_checkpoints()
        self.get_logger().info(
            'judge_system ready (world=%s, lap_count=%d, total_doors=%d)'
            % (self.get_parameter('world').value,
               self.get_parameter('lap_count').value, self._total_doors))

    # ---- 工具 ----

    def _now(self):
        """当前 ROS 时钟（秒，float）。"""
        return self.get_clock().now().nanoseconds / 1e9

    # ---- 初始化 ----

    def _load_checkpoints(self):
        world = self.get_parameter('world').value
        try:
            pkg_share = get_package_share_directory("tianracer_gazebo")
        except Exception:
            pkg_share = None
        candidates = []
        if pkg_share:
            candidates.append(os.path.join(pkg_share, "waypoint_race"))
        candidates.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "waypoint_race"))
        for d in candidates:
            f = os.path.join(d, f"{world}_check_points.yaml")
            if os.path.isfile(f):
                position_check.load_checkpoint(f)
                return
        self.get_logger().warn(f"check_points.yaml for world '{world}' not found")

    # ---- 模型校验 ----

    def _model_check(self):
        try:
            pkg_share = get_package_share_directory("tianracer_gazebo")
        except Exception:
            pkg_share = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "share", "tianracer_gazebo")
        passed, detail = md5_check.verify_hash(pkg_share)
        if passed:
            self.get_logger().info(".........................Model Checking Passed.......................")
        else:
            self.get_logger().warn(".........................Model Checking Unpassed.....................")
        for rel, d in detail.items():
            if d['status'] in ('missing', 'tampered'):
                self.get_logger().error(f"  [{d['status']}] {rel}")
        self._check_flag = passed
        return passed

    # ---- 回调 ----

    def _update_callback(self, msg):
        """odom 回调：更新位置、停车检测、检查点判定、评分。"""
        # 里程计增量（odom 系，用于停车检测与累计距离）
        pos = msg.pose.pose.position
        self._nw_time = self._now()

        step = 0.0
        if self._last_position is not None:
            step = math.hypot(pos.x - self._last_position[0], pos.y - self._last_position[1])
            self._move_dis += step
        self._last_position = (pos.x, pos.y)

        if not self._start_flag or self._stop_flag:
            return

        # 车动驱动计时：按节流周期发布，使窗口用时随行驶连续更新（不必等到过门）
        if self._nw_time - self._last_pub_time >= self.get_parameter('pub_interval').value:
            self._last_pub_time = self._nw_time
            self._publish_score()

        # 停车检测：持续未动超时判犯规
        if step < 0.001:
            if self._wait_check_timing is None:
                self._wait_check_timing = self._nw_time
            elif self._nw_time - self._wait_check_timing > self.get_parameter('stop_time_threshold').value:
                self.get_logger().warn("Checking the car has not moved for a long time")
                self._finalize(reason='stopped')
        else:
            self._wait_check_timing = None

        # 超时未过门：车在动但绕圈/定位漂移时停车判罚不会触发，靠这条兜底终止
        if self._stop_flag:
            return
        checkpoint_timeout = self.get_parameter('checkpoint_timeout').value
        if checkpoint_timeout > 0 and self._nw_time - self._temp_time > checkpoint_timeout:
            self.get_logger().warn(
                "No checkpoint detected for over %.0fs, terminating judge" % checkpoint_timeout)
            self._finalize(reason='stopped')
            return

        # 检查点判定：转 map 系与检查门统一坐标系
        x, y = self._map_position()
        if x is None:
            return
        passed, _dist = position_check.analysis((x, y))
        if passed is not None:
            self._on_checkpoint_passed()

    def _map_position(self):
        """经 TF 查 map->base_footprint，返回 (x, y)；TF 未就绪返回 (None, None)。"""
        try:
            trans = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
            t = trans.transform.translation
            return t.x, t.y
        except Exception:
            return None, None

    def _on_checkpoint_passed(self):
        """通过一个检查门：按门分值表累加门分，再判定完赛。"""
        self._pass_point_idx = position_check.ana_cnt

        # 本段距离 / 时间（上一门 -> 本门），仅用于日志诊断
        seg_dis = self._move_dis - self._seg_move_dis_start
        seg_time = self._nw_time - self._temp_time
        self._seg_move_dis_start = self._move_dis
        self._temp_time = self._nw_time

        # 门分：position_check.ana_cnt 首个门即为 1，故减 1 取表
        door_score = self._door_scores[self._pass_point_idx - 1]
        self._scores += door_score

        self.get_logger().info(
            "通过第 %d 个门 +%.0f 分（门分累计 %.0f）| 段距离=%.2fm 段用时=%.2fs 段速=%.2fm/s"
            % (self._pass_point_idx, door_score, self._scores,
               seg_dis, seg_time, (seg_dis / seg_time) if seg_time > 0 else 0.0))

        if self._pass_point_idx >= self._total_doors:
            self._finish()
        else:
            self._publish_score()

    # ---- 评分（原版公式）----

    def _cal_racing_score(self):
        """原版 Judge.cal_racing_score：总分 = 门分累计 + 速度分，返回总分。

        原版的总分取 3 位小数（round 而非截断），此处对齐。

        注意 _speed_score 平时恒为 0，只在完赛结算时算一次（见 _finalize）——这是原版
        的语义：界面的分数标签绑的是 self.scores，速度分只在过终点那一次并入
        scores。实测（noetic 容器跑原版 .so）：比赛中 times=90.06 时界面显示 25
        （纯门分），而 25 + 35*42/90.06 ≈ 41.3；提前终止时界面上仍是门分。
        """
        self._total_score = round(self._scores + self._speed_score, 3)
        return self._total_score

    def _compute_speed_score(self):
        """算速度分：speed_score_max * min(1, 3 * score_alpha / times)。

        跑进 3 * score_alpha（=42s）拿满，超出后按 42/t 衰减。只在完赛结算时调用一次。
        """
        times = self._nw_time - self._start_time
        if not self._start_time or times <= 0:
            self._speed_score = 0.0       # 未开表（idle / 刚 reset）：不计速度分
        else:
            alpha = self.get_parameter('score_alpha').value
            speed_max = self.get_parameter('speed_score_max').value
            self._speed_score = speed_max * min(1.0, 3.0 * alpha / times)
        return self._speed_score

    def _publish_score(self, final=False):
        self._cal_racing_score()
        msg = String()
        msg.data = (
            "state: %s | score: %.3f | door_points: %.0f | speed_score: %.3f | "
            "points: %d/%d | move_dis: %.2fm | elapsed: %.2fs"
            % (self._state, self._total_score, self._scores, self._speed_score,
               self._pass_point_idx, self._total_doors, self._move_dis,
               self._nw_time - self._start_time if self._start_time else 0.0))
        self._score_pub.publish(msg)
        d = Float32()
        d.data = float(self._move_dis)
        self._move_dis_pub.publish(d)
        if final:
            self.get_logger().info("===== FINAL " + msg.data + " =====")

    # ---- 主程序（竞速状态机）生命周期 ----
    # 对齐原版：裁判是主程序的唯一持有者——「启动」时拉起，重置/结束时杀掉。
    # 这样界面上的「目标代码已启动」才有对应语义，主程序也无需自己订阅启动信号。

    RACER_PKG = 'tianracer_gazebo'
    RACER_EXEC = 'f1tenth_racer.py'

    def _spawn_racer(self):
        """拉起主程序；已在运行则不重复拉起。"""
        if self._racer_proc is not None and self._racer_proc.poll() is None:
            self.get_logger().info('racer already running (pid=%d)' % self._racer_proc.pid)
            return
        cmd = ['ros2', 'run', self.RACER_PKG, self.RACER_EXEC,
               '--ros-args', '-p', 'use_sim_time:=true']
        if self._ns:
            # 等价原版命令尾部的 __ns:=<ns>
            cmd.append(f'__ns:={self._ns}')
        env = dict(os.environ)
        # 主程序从环境变量取 world（模块级 os.getenv），由裁判的 world 参数统一决定
        env['TIANRACER_WORLD'] = self.get_parameter('world').value
        try:
            self._racer_proc = subprocess.Popen(cmd, env=env, start_new_session=True)
            self.get_logger().info(
                'racer started (pid=%d): %s' % (self._racer_proc.pid, ' '.join(cmd)))
        except Exception as e:
            self._racer_proc = None
            self.get_logger().error(f'racer start failed: {e}')

    def _find_racer_pids(self):
        """扫描 /proc 找出命令行以 RACER_EXEC 结尾的进程（跳过自身）。

        替代原版的 `ps aux | grep f1tenth_racer | grep -v grep`：管道里写着目标名，
        命令行会自匹配到管道自身，属于已多次踩过的坑，故改为读 /proc 且不经过 shell。
        """
        pids = []
        me = os.getpid()
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            pid = int(entry)
            if pid == me:
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    argv = [a for a in f.read().decode('utf-8', 'replace').split('\x00') if a]
            except Exception:
                continue
            if any(a.endswith(self.RACER_EXEC) for a in argv):
                pids.append(pid)
        return pids

    def _kill_racer(self):
        """终止主程序。

        先收拾 Popen 句柄（连同其进程组），再扫 /proc 清理残留——裁判自身重启过时
        句柄会丢，残留进程就成了孤儿，与原版用 ps 扫描的动机一致。
        """
        proc, self._racer_proc = self._racer_proc, None
        if proc is not None and proc.poll() is None:
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(os.getpgid(proc.pid), sig)
                except Exception:
                    try:
                        proc.send_signal(sig)
                    except Exception:
                        pass
                try:
                    proc.wait(timeout=2.0)
                    break
                except Exception:
                    continue
            self.get_logger().info('racer stopped (pid=%d)' % proc.pid)
        for pid in self._find_racer_pids():
            try:
                os.kill(pid, signal.SIGKILL)
                self.get_logger().warn(f'killed leftover racer pid={pid}')
            except Exception:
                pass

    # ---- 控制服务 ----

    def _start_cb(self, req, resp):
        self.get_logger().info("Model Checking....................................")
        if not self._model_check():
            self.get_logger().error("Model Checking Unpassed, judge aborted")
            return resp
        self._start_flag = True
        self._stop_flag = False
        self._state = 'running'
        # 先拉起主程序再开表：给 Nav2 首次规划留出时间，避免开局就压在停车判罚阈值上
        self._spawn_racer()
        self._start_time = self._now()
        self._temp_time = self._start_time
        self._nw_time = self._start_time
        self._last_pub_time = self._start_time
        self._seg_move_dis_start = 0.0
        self._wait_check_timing = None
        # 立即发布一次：点「启动」后窗口即可看到计时起点，无需等车动
        self._publish_score()
        self.get_logger().info("md5 check finished....................................")
        self.get_logger().info("Judge started")
        return resp

    def _stop_cb(self, req, resp):
        if self._start_flag and not self._stop_flag:
            self._finalize(reason='stopped')
        return resp

    def _finish(self):
        """完赛：结算。"""
        total_time = self._nw_time - self._start_time
        self.get_logger().info(
            "===== Race Finished! total_time=%.2fs door_points=%.0f ====="
            % (total_time, self._scores))
        self._finalize(reason='finished')

    def _finalize(self, reason):
        """统一停止/结算入口。reason: 'finished' | 'stopped'。"""
        self._stop_flag = True
        self._state = reason
        # 速度分只在完赛结算时并入一次（原版语义）：提前终止报的就是纯门分
        if reason == 'finished':
            self._compute_speed_score()
        self._publish_score(final=True)
        # 对齐原版 terminate_test：结束时停掉主程序，不再下发新目标
        self._kill_racer()
        if reason == 'finished':
            self.get_logger().info("Car finished the race")
        else:
            self.get_logger().info("Car stopped, Judge_system process has ended")
        self.get_logger().info("[32mIf you want to restart Car, please use reset function")

    def _brake_cb(self, req, resp):
        self._brake()
        return resp

    def _brake(self):
        """刹停（等价 ROS1 reset_racecar 里的 rosservice call <ns>/emergency_brake）。"""
        if self._brake_client.service_is_ready():
            self._brake_client.call_async(Empty.Request())
            self.get_logger().info("brake the racecar")
        else:
            self.get_logger().warn("emergency_brake service not available")

    def _reset_cb(self, req, resp):
        """重置：按原版 reset_racecar 的动作序列把比赛环境复位。

        原版 reset_racecar 依次执行五个动作：
          ① rosservice call /gazebo/reset_world          模型回出生位姿
          ② rosrun tianracer_gazebo initialpose_pub.py   重发初始位姿
          ③ kill f1tenth_racer                           停主程序
          ④ rosservice call <ns>/emergency_brake         刹停
          ⑤ rosservice call <ns>/move_base/clear_costmaps 清代价地图
        本移植版另加取消 Nav2 目标（等价原版的独立命令 movebase_goal_cancel）。

        与原版的唯一顺序差异：initialpose 不紧跟在 reset_world 后面同步发，而是等传送
        确认 + 一段沉降时间后再发（_schedule_init_pose）。原版 reset_world 是同步阻塞的
        rosservice call，回来时位姿已生效；本移植版的 set_pose 是异步的，紧跟着发会让
        AMCL 拿传送前的 odom 定位，或直接被 MessageFilter 丢弃。
        """
        self.get_logger().info("reset_racecar")
        # initialpose 不在这里发：必须等 _reset_world 的传送确认后再发，见
        # _on_reset_world_done / _schedule_init_pose。
        self._reset_world()
        # 先杀主程序再取消目标：否则它可能在本函数返回后再下发一个新目标
        self._kill_racer()
        self._cancel_nav_goals()
        self._brake()
        self._clear_costmaps()
        # 重置裁判状态
        position_check.reset_variables()
        self._start_flag = False
        self._stop_flag = False
        self._state = 'idle'
        self._pass_point_idx = 0
        self._move_dis = 0.0
        self._start_time = 0.0
        self._temp_time = 0.0
        self._seg_move_dis_start = 0.0
        self._scores = 0.0
        self._speed_score = 0.0
        self._total_score = 0.0
        self._wait_check_timing = None
        self._last_position = None
        self._last_pub_time = 0.0
        # 发布一条全零载荷（state: idle）：窗口等到新消息才会刷新数字，
        # 不发布则上一次的分数/用时会一直挂在界面上，与「已重置」的语义矛盾。
        self._publish_score()
        self.get_logger().info("Judge reset done")
        return resp

    # ---- reset 的各步动作 ----

    def _reset_world(self):
        """把车送回出生位姿（等价 ROS1 的 rosservice call /gazebo/reset_world）。

        用 gz 的 /world/default/set_pose 而不是 /world/default/control 的 reset：
        实测 gz-sim 6.18 下 ControlWorld 的 reset（model_only 与 all 都试过）返回 success
        却不会改动运行时 create 出来的模型位姿，而 all 还会把仿真时间清零（Nav2/tf 的
        sim clock 倒退，运行中的节点时序被打乱）。set_pose 能精确设定位姿且不动时间。

        与 _cancel_nav_goals 同理，不能在本回调内 spin 等响应（执行器正在跑本回调），
        故挂 done 回调后立即返回。
        """
        if not self._set_pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'reset world failed: /world/default/set_pose not available')
            # 传送没做成也要让 AMCL 回起点，否则定位会一直停在旧位姿
            self._schedule_init_pose()
            return
        req = SetEntityPose.Request()
        req.entity.name = self.GZ_MODEL_NAME
        req.entity.type = Entity.MODEL
        req.pose.position.x = self.get_parameter('init_x').value
        req.pose.position.y = self.get_parameter('init_y').value
        req.pose.position.z = self.get_parameter('init_z').value
        quat = quaternion_from_euler(0.0, 0.0, self.get_parameter('init_yaw').value)
        req.pose.orientation.x = quat[0]
        req.pose.orientation.y = quat[1]
        req.pose.orientation.z = quat[2]
        req.pose.orientation.w = quat[3]
        fut = self._set_pose_client.call_async(req)
        fut.add_done_callback(self._on_reset_world_done)

    def _on_reset_world_done(self, fut):
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().warn('reset world failed: %s' % e)
            self._schedule_init_pose()
            return
        if resp is None or not resp.success:
            self.get_logger().warn('reset world failed: gz returned no success')
        else:
            self.get_logger().info('reset world: car back to start pose')
        self._schedule_init_pose()

    def _schedule_init_pose(self):
        """传送确认后延迟 INITPOSE_SETTLE_SEC 再发 initialpose（一次性定时器）。

        不在回调里 sleep：执行器正跑本回调，阻塞会连带卡住 odom/检查门判定。
        """
        if self._initpose_timer is not None:
            self.destroy_timer(self._initpose_timer)
        self._initpose_timer = self.create_timer(
            self.INITPOSE_SETTLE_SEC, self._on_init_pose_timer)

    def _on_init_pose_timer(self):
        self.destroy_timer(self._initpose_timer)
        self._initpose_timer = None
        self._publish_init_pose()

    def _publish_init_pose(self):
        """重发初始位姿，让 AMCL 跟着车回到起点（等价 ROS1 reset 里的 initialpose_pub.py）。

        模型被送回出生点后，AMCL 的粒子仍停在旧位姿上，不重发则下一场比赛的导航会以
        错误定位规划。frame_id 用 <ns>/map——与 Nav2 参数里的 global_frame_id 一致，
        AMCL 的 initialpose 订阅带 MessageFilter，frame_id 对不上会被直接丢弃。

        stamp 回退 INITPOSE_STAMP_BACKDATE：用 now() 时它可能比最新的
        odom->base_footprint 超前约一个发布周期（实测超前 10ms），MessageFilter 查不到
        对应时刻的 TF 就把整条消息丢掉，AMCL 定位不更新（实测后果：规划器出不了路径、
        车不动、被 10s 停车判罚终止）。回退后落点必在 TF 缓存内，且此时车已静止，
        用稍早的 odom 不影响结果。
        """
        msg = PoseWithCovarianceStamped()
        now_ns = self.get_clock().now().nanoseconds
        backdate_ns = int(self.INITPOSE_STAMP_BACKDATE * 1e9)
        msg.header.stamp = rclpy.time.Time(
            nanoseconds=now_ns - backdate_ns if now_ns > backdate_ns else now_ns).to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = self.get_parameter('init_x').value
        msg.pose.pose.position.y = self.get_parameter('init_y').value
        yaw = self.get_parameter('init_yaw').value
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        # 与 initialpose_pub.py 一致的协方差（x/y/z 与三个角度的方差均为 0.1）
        for i in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[i] = 0.1
        self._initpose_pub.publish(msg)
        self.get_logger().info(
            'Published initialpose (x=%.2f y=%.2f yaw=%.2f)'
            % (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    def _clear_costmaps(self):
        """清空 Nav2 全局/局部代价地图（等价 ROS1 <ns>/move_base/clear_costmaps）。"""
        for client in self._clear_costmap_clients:
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    'clear costmap failed: %s not available' % client.srv_name)
                continue
            fut = client.call_async(ClearEntireCostmap.Request())
            fut.add_done_callback(
                lambda f, name=client.srv_name: self._on_clear_costmap_done(f, name))

    def _on_clear_costmap_done(self, fut, name):
        try:
            fut.result()
        except Exception as e:
            self.get_logger().warn('clear costmap %s failed: %s' % (name, e))
        else:
            self.get_logger().info('cleared costmap: %s' % name)

    def _cancel_nav_goals(self):
        """取消 Nav2 当前全部导航目标（等价 ROS1 的 movebase_goal_cancel）。

        ROS2 的 ActionClient 没有 cancel_all_goals_async（该方法不存在，旧写法每次都
        抛 AttributeError 被这里吞掉，日志只留一条 warn——重置时目标其实从没被取消过）。
        正确做法是调用 action 的隐藏服务 <action>/_action/cancel_goal，
        请求里 goal_id 传全零 UUID 表示「取消该 action server 上的全部目标」。

        注意不能用 rclpy.spin_until_future_complete 等响应：本函数由 _reset_cb 调用，
        而执行器正在跑这个回调，重入 spin 拿不到任何响应——实测恒超时 2.000s 且
        result() 为 None（同一调用放在回调外 0.001s 即返回），于是 reset 表面成功、
        目标其实从没被取消，比赛途中按 reset 车会继续跑向旧目标。
        故挂 done 回调后立即返回，由执行器在后续 spin 中投递响应并打日志。
        """
        if not self._cancel_goal_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'cancel nav goals failed: cancel_goal service not available')
            return
        # goal_info 用默认构造（全零 UUID）= 取消全部目标
        fut = self._cancel_goal_client.call_async(CancelGoal.Request())
        fut.add_done_callback(self._on_cancel_goals_done)

    def _on_cancel_goals_done(self, fut):
        """cancel_goal 响应到达后的日志（由执行器在后续 spin 中回调）。"""
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().warn('cancel nav goals failed: %s' % e)
            return
        if resp is None:
            self.get_logger().warn('cancel nav goals failed: no response')
        elif resp.return_code != CancelGoal.Response.ERROR_NONE:
            self.get_logger().warn(
                'cancel nav goals failed: return_code=%d' % resp.return_code)
        else:
            self.get_logger().info(
                'cancel nav goals: %d goal(s) canceled' % len(resp.goals_canceling))


def main(args=None):
    rclpy.init(args=args)
    node = Judger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 裁判退出时一并收掉主程序，避免留下孤儿进程（原版 on_closing 亦如此）
        node._kill_racer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
