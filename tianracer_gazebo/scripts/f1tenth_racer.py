#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_gazebo/scripts/f1tenth_racer.py) -> ROS 2 Humble 移植。
# 竞速状态机：按 waypoint yaml 依次发 Nav2 导航目标，接近后进入下一目标。
# move_base actionlib -> Nav2 NavigateToPose action；close_to_send 用 tf2 查 map->base_footprint。

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray
import tf2_ros
from tf2_ros import Buffer, TransformListener

import waypoint_race.utils as utils

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")
robot_name = os.getenv("TIANBOT_NAME", os.getenv("TIANRACER_NAME", ""))


class RaceStateMachine(Node):
    CHECK_INTERVAL = 0.1        # 秒；判定与日志都按 10 Hz
    # 到点判定阈值（米）。若用逐轴方框比较（阈值 2.15）：
    #   abs(gx-tx) < 2.15 and abs(gy-ty) < 2.15
    # 对角可达 3.04m。本赛道净宽约 1m、路点间距 3-8m，逐轴方框比较会让车距目标还有 2m 上下
    # 就被判「到点」并推进路点，导致实际路线与设计圈线脱节
    # （跑 59.56m 仍只计到 2/9 门、从未穿过 gate2 闭合那一圈）。
    # 改为真半径判定并收紧到 1.0m。
    APPROACH_THRESHOLD = 1.0
    STATS_PERIOD = 5.0          # 秒；状态机计数汇总周期

    # spin 节流参数，取值依据见 _spin_drain
    SPIN_DRAIN_TIMEOUT = 0.001  # 秒；排空时单次 spin_once 的等待上限
    SPIN_IDLE_SLEEP = 0.02      # 秒；排空后的固定睡眠
    SPIN_MAX_DRAIN = 64         # 单轮排空最多处理的回调数（安全上限）

    def __init__(self, filename, repeat=True):
        super().__init__('f1tenth_racer')
        self._waypoints = utils.get_waypoints(filename)

        self._ac_move_base = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Wait for navigate_to_pose server')
        self._counter = 0
        self._repeat = repeat

        self._pub_viz_marker = self.create_publisher(MarkerArray, 'viz_waypoints', 1)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._goal_handle = None
        self._result_future = None
        self._goal_pending = False

        # 状态机计数：用于核对「到点次数 vs 下发目标次数」——仅靠日志无法判定，
        # 故把各分支的进入次数显式计数并周期输出。
        self._stats = {'move_to_next': 0, 'skip': 0, 'goal_sent': 0, 'accepted': 0,
                       'rejected': 0, 'accept_timeout': 0, 'close': 0, 'abort': 0}
        self.create_timer(self.STATS_PERIOD, self._log_stats)

    def _log_stats(self):
        self.get_logger().info('状态机计数: ' + ' '.join(
            '%s=%d' % (k, v) for k, v in self._stats.items()))

    def _spin_drain(self):
        """处理完已就绪的回调后节流返回；无待处理工作时立即返回，不做忙轮询。

        不能写成 `while rclpy.ok(): rclpy.spin_once(self, timeout_sec=X)`：只要队列里
        始终有消息，spin_once 就不会等满超时而是立刻返回，循环退化成忙轮询。节点以
        use_sim_time=true 运行时，rclpy 的 TimeSource 会自动订阅 /clock，gz-sim 下该
        话题约 960Hz，于是循环以消息到达速率空转。

        改为「排空 + 固定睡眠」：连续 spin_once，直到某次真的等满 SPIN_DRAIN_TIMEOUT
        （说明此刻没有待处理工作）再睡 SPIN_IDLE_SLEEP，把循环速率与消息速率解耦。
        SPIN_DRAIN_TIMEOUT 取 1ms，略小于 /clock 的到达间隔（约 1.04ms），使排空既能
        吃掉积压又能在空档立刻退出；SPIN_IDLE_SLEEP 取 20ms，即新消息最坏延迟约
        20ms，远小于调用方 10Hz 的判定周期。

        /clock 的订阅 QoS 是 depth=1 + BEST_EFFORT，睡眠期间到达的旧时钟消息会被直接
        覆盖、只留最新一条；时间源本就只需要最新值，因此既不堆积也不丢语义。
        SPIN_MAX_DRAIN 只是安全上限：若某话题快到排空永不退出，本轮也会在处理 64 个
        回调后进入睡眠，不会饿死其它话题。

        排空超时不能取得更小：过小时单轮只处理约一个回调，低频话题会被 /clock 挤掉
        一半的投递 —— 故取 1ms。
        """
        for _ in range(self.SPIN_MAX_DRAIN):
            started = time.monotonic()
            rclpy.spin_once(self, timeout_sec=self.SPIN_DRAIN_TIMEOUT)
            if time.monotonic() - started >= self.SPIN_DRAIN_TIMEOUT * 0.9:
                break                       # 等满超时 = 此刻没有待处理工作
            if not rclpy.ok():
                return
        time.sleep(self.SPIN_IDLE_SLEEP)

    def move_to_next(self):
        self._stats['move_to_next'] += 1
        # 并发守卫：上一目标仍在执行时先取消，避免 Nav2 目标抢占卡死
        if self._goal_pending:
            self._cancel_pending_goal()
        # 清掉上一轮目标的 result future，防止 close_to_send 误读旧结果导致级联跳过
        self._goal_pending = False
        self._goal_handle = None
        self._result_future = None

        pos = self._get_next_destination()
        if pos is None:
            self.get_logger().info("Finishing Race")
            return True

        # 跳过已在阈值内的路点（如 spawn 点附近的 P0），避免无意义抢占
        if self._within_threshold(pos):
            self._stats['skip'] += 1
            self.get_logger().info(
                "Skip %s (already within %.2f m)" % (pos['name'], self.APPROACH_THRESHOLD))
            return False

        # 目标须带时间戳：每次都用 rospy.Time.now()，移植时丢掉了，
        # header.stamp 变成 0。节点以 use_sim_time=true 运行（由裁判拉起时传入），
        # 故此处取仿真时钟，与 Nav2 各节点一致。
        goal = utils.create_nav_goal(pos, stamp=self.get_clock().now().to_msg())
        self.get_logger().info("Move to %s" % pos['name'])

        if not self._ac_move_base.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server not available")
            return False

        self._goal_pending = True
        self._stats['goal_sent'] += 1
        self._send_goal_future = self._ac_move_base.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        # 等待目标被接受（goal_handle 就绪），再进入接近检测，否则 _result_future 仍是空的
        if not self._wait_for_acceptance():
            return False
        self.close_to_send(goal)
        return False

    def _wait_for_acceptance(self):
        """阻塞等待目标被 accept（goal_handle 与 result_future 就绪），超时或被拒返回 False。

        超时用墙钟 time.monotonic() 而不是 self.get_clock()：节点刚启动时还没收到
        /clock，仿真时钟 now() 返回 0，deadline 被算成 0+10s；等 /clock 一到、now()
        跳当前仿真时刻（数十秒）后 now() > deadline 立刻成立，目标会在发出后零点几秒
        就被误判超时放弃（发车线的第一个路点会被跳过）。
        """
        deadline = time.monotonic() + 10.0
        while rclpy.ok():
            self._spin_drain()
            if self._goal_handle is not None:
                return True
            if not self._goal_pending:
                return False  # 目标被拒
            if time.monotonic() > deadline:
                self._stats['accept_timeout'] += 1
                self.get_logger().warn("goal acceptance timeout")
                return False

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._stats['rejected'] += 1
            self.get_logger().info("Goal rejected")
            self._goal_pending = False
            return
        self._stats['accepted'] += 1
        self.get_logger().info("Goal accepted")
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()

    def _cancel_pending_goal(self):
        """取消当前 Nav2 目标，并等它在 Nav2 侧真正收尾后再返回。

        rclpy 的取消挂在 goal handle 上（ClientGoalHandle.cancel_goal_async）；
        ActionClient 既没有 cancel_goal_async 也没有 cancel_all_goals_async，写错会直接
        抛 AttributeError。且取消失败也必须清掉 _goal_pending——否则路点推进会被永久
        卡在同一个目标上（反复刷「close to goal」而永不推进）。

        为什么必须等：调用方（close_to_send → move_to_next）紧接着就下发下一个目标，
        而 Nav2 的取消是异步的——请求到达时「当前活动目标」很可能已经是新目标，
        bt_navigator 会把刚接受的新目标一起 Aborting（Nav2 日志表现为
        Client requested to cancel the goal → Aborting handle 打在新路点上），
        车停在原地 10s 后被裁判判停车犯规终止比赛。
        注意只等 cancel 的响应不够：它可能在毫秒级就返回、远早于 Nav2 处理完；
        要等的是旧目标的 result future 完成，那才代表 Nav2 已把它彻底收尾。
        """
        if self._goal_handle is None or not self._goal_pending:
            return
        self.get_logger().info("Cancel pending goal")
        try:
            self._goal_handle.cancel_goal_async()
            if self._result_future is not None:
                rclpy.spin_until_future_complete(
                    self, self._result_future, timeout_sec=2.0)
        except Exception as e:
            self.get_logger().warn("cancel goal failed: %s" % e)
        finally:
            self._goal_pending = False

    def _within_threshold(self, pos):
        """检测当前位置是否已处于路点阈值内。"""
        base_frame = f"{robot_name}/base_footprint" if robot_name else "base_footprint"
        try:
            trans = self._tf_buffer.lookup_transform("map", base_frame, rclpy.time.Time())
            gx = pos['pose']['position']['x']
            gy = pos['pose']['position']['y']
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            return math.hypot(gx - tx, gy - ty) < self.APPROACH_THRESHOLD
        except Exception:
            return False

    def close_to_send(self, goal):
        """
        等车进入目标点阈值（APPROACH_THRESHOLD，1.0m）后推进下一个路点。

        到点判定只看距离，不看 move_base 的结果状态。此处保持同一语义：Nav2 目标
        异常结束（ABORTED/FAILED）时只告警、**不**推进路点——早先把「目标结束」
        一律当成功，导致规划失败时静默跳过全部路点空转（车原地不动却在循环）。
        改为等待后，若车始终无法接近，裁判的停车判罚会介入并终止比赛。
        """
        base_frame = f"{robot_name}/base_footprint" if robot_name else "base_footprint"
        last_check = 0.0
        warned = False
        while rclpy.ok():
            # 先排空回调（保证 action 回调与 TF 及时），再按 10 Hz 做 TF 查询与日志，
            # 按 10 Hz 节流，避免每轮都打日志。
            self._spin_drain()
            now = time.monotonic()
            if now - last_check < self.CHECK_INTERVAL:
                continue
            last_check = now

            if self._result_future is not None and self._result_future.done() and not warned:
                self._goal_pending = False
                self._stats['abort'] += 1
                try:
                    status = self._result_future.result().status
                except Exception:
                    status = "?"
                self.get_logger().warn(
                    "Nav2 目标异常结束 (status=%s)，不推进路点，继续按 %.2fm 阈值等待"
                    % (status, self.APPROACH_THRESHOLD))
                warned = True
            # try 只包住 TF 查询：到点判定与取消目标必须留在 try 之外，
            # 否则里面的任何异常（例如取消 API 写错抛 AttributeError）都会被
            # 下面的 except 吞掉，导致 return True 永远执行不到、路点永不推进。
            try:
                trans = self._tf_buffer.lookup_transform(
                    "map", base_frame, rclpy.time.Time())
            except Exception:
                continue
            gx = goal.pose.pose.position.x
            gy = goal.pose.pose.position.y
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            dist = math.hypot(gx - tx, gy - ty)
            self.get_logger().info(
                "相对坐标:(%.2f,%.2f) goal坐标:(%.2f,%.2f) 距离=%.2fm" % (tx, ty, gx, gy, dist))
            if dist < self.APPROACH_THRESHOLD:
                self._stats['close'] += 1
                self.get_logger().info("close to goal (dist=%.2fm)" % dist)
                self._cancel_pending_goal()
                return True
        return False

    def _get_next_destination(self):
        if self._counter >= len(self._waypoints):
            if self._repeat:
                self._counter = 0
            else:
                return None
        next_destination = self._waypoints[self._counter]
        self._counter += 1
        return next_destination


def main(args=None):
    rclpy.init(args=args)

    filename = os.getenv("RACE_WAYPOINT_FILE", "")
    if not filename:
        import ament_index_python.packages as aip
        pkg_path = aip.get_package_share_directory("tianracer_gazebo")
        filename = os.path.join(pkg_path, "waypoint_race", f"{world}_points.yaml")

    node = RaceStateMachine(filename, repeat=True)
    try:
        while rclpy.ok():
            finished = node.move_to_next()
            if finished:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
