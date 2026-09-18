#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
建图巡游驱动：依次把「竞速路点 + 检查门端点」发成 Nav2 目标，让车走遍赛道。

配套 tools/mapping_round.sh 使用（该脚本起 gz + slam_toolbox + Nav2 后调用本脚本）。

用法：
  python3 mapping_drive.py <points.yaml> <check_points.yaml> [out.log]

设计要点：
  · 用 Nav2 的 NavigateToPose action（与竞速同一条控制链路），保证建图时车走的路
    与竞速时一致 —— 这正是要把「竞速会经过的区域」建进地图的原因。
  · 每个目标设**独立超时**（默认 90s）：某个点够不到不应拖垮整轮建图。
  · 不设朝向要求（用 theta 容差放大的 goal checker 行为）——建图只看位置覆盖，
    强求朝向会让近目标点反复掉头、浪费里程。
  · 结束条件：所有目标走完，或总超时（由外层 timeout 900 兜底）。
"""

import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

GOAL_TIMEOUT = 90.0     # 单个目标的墙钟超时（秒）
SETTLE_TIME = 1.5       # 到达后等待地图更新的时间（秒）


def load_points(points_file, check_file):
    """读取路点与检查门端点，返回 [(x, y, 标签), ...]（已去重、保持顺序）。"""
    pts, seen = [], set()

    def add(x, y, label):
        key = (round(x, 2), round(y, 2))
        if key in seen:
            return
        seen.add(key)
        pts.append((x, y, label))

    for f, prefix in ((points_file, 'wp'), (check_file, 'gate')):
        if not f:
            continue
        try:
            d = yaml.safe_load(open(f, encoding='utf-8'))
        except Exception as e:
            print('读取 %s 失败: %s' % (f, e))
            continue
        for i, p in enumerate(d.get('waypoints', [])):
            pos = p['pose']['position']
            add(pos['x'], pos['y'], '%s%d' % (prefix, i))
    return pts


def make_pose(x, y):
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = 0.0
    m.pose.orientation.w = 1.0
    return m


class MappingDriver(Node):
    def __init__(self):
        super().__init__('mapping_driver')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait_server(self, timeout=60.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def goto(self, x, y, timeout=GOAL_TIMEOUT):
        """前往 (x,y)；返回 (是否到达, 用时秒, 说明)。"""
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y)
        t0 = time.time()

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, time.time() - t0, '目标未被接受'

        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=timeout)
        el = time.time() - t0
        if not res_future.done():
            handle.cancel_goal_async()
            return False, el, '超时 %.0fs 未到达' % timeout

        status = res_future.result().status
        # 4 = STATUS_SUCCEEDED
        return (status == 4), el, ('到达' if status == 4 else '终止(status=%d)' % status)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    points_file, check_file = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else None

    pts = load_points(points_file, check_file)
    if not pts:
        print('!! 未读到任何目标点')
        return 1

    logf = open(out, 'w', encoding='utf-8') if out else None

    def log(msg):
        print(msg, flush=True)
        if logf:
            logf.write(msg + '\n')
            logf.flush()

    rclpy.init()
    node = MappingDriver()
    log('等待 navigate_to_pose action server ...')
    if not node.wait_server(120.0):
        log('!! action server 未就绪')
        node.destroy_node(); rclpy.shutdown()
        return 1
    log('server 就绪，开始巡游 %d 个目标' % len(pts))

    n_ok = 0
    t_start = time.time()
    for i, (x, y, label) in enumerate(pts, 1):
        log('[%2d/%2d] %-8s -> (%7.2f, %7.2f)' % (i, len(pts), label, x, y))
        ok, el, why = node.goto(x, y)
        log('        %s  用时 %.1fs  %s' % ('✔' if ok else '✘', el, why))
        if ok:
            n_ok += 1
            time.sleep(SETTLE_TIME)
        else:
            # 失败点也等一会：slam_toolbox 可能仍在该区域积累
            time.sleep(0.5)

    total = time.time() - t_start
    log('巡游完成: %d/%d 个目标到达，总用时 %.1fs' % (n_ok, len(pts), total))

    node.destroy_node()
    rclpy.shutdown()
    if logf:
        logf.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
