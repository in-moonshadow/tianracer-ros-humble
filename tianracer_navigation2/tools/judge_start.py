#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发 /start 并记录「请求真正送达服务端」的时刻（替代 ros2 service call CLI）。

为什么需要它：dwb_round.sh / teb_round.sh 原先用 `ros2 service call /start ...`，
那会新起一个 Python 进程（实测最轻的 ros2 CLI 命令也要 0.30~0.36s，服务调用还要加
发现与往返），这段开销被算进了「起步死时间」。真实用户点 judge_display.py 的按钮走
的是**常驻 rclpy client**（call_async，不启进程），没有这部分。故本脚本用同样方式
发请求，让 t_start.txt 记录的才是真实体验的起点。

用法（由 dwb_round.sh / teb_round.sh 调用）：
    python3 judge_start.py <t_start.txt 路径>
退出码：0 = 收到响应；1 = 失败（未就绪/超时）。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

WAIT_READY = 20.0   # 等服务可用的上限(s)
WAIT_RESP = 10.0    # 等响应的上限(s)


def main():
    if len(sys.argv) < 2:
        print('用法: judge_start.py <t_start.txt>', file=sys.stderr)
        return 2
    out_path = sys.argv[1]

    rclpy.init()
    node = Node('judge_start_client')
    try:
        cli = node.create_client(Empty, '/start')

        deadline = time.monotonic() + WAIT_READY
        while not cli.service_is_ready():
            if time.monotonic() > deadline:
                print('!! /start 服务在 %.0fs 内未就绪' % WAIT_READY, file=sys.stderr)
                return 1
            time.sleep(0.02)

        # 服务就绪后再记时刻：这才是「请求发出」的真实起点
        t_send = time.time()
        with open(out_path, 'w') as f:
            f.write('%.6f\n' % t_send)
        print('/start 发出 @ %.6f' % t_send, flush=True)

        fut = cli.call_async(Empty.Request())
        t0 = time.monotonic()
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() - t0 > WAIT_RESP:
                print('!! /start 响应超时（%.0fs）' % WAIT_RESP, file=sys.stderr)
                return 1
        dt = time.time() - t_send
        print('/start 返回 @ %.6f（往返 %.3fs）' % (time.time(), dt), flush=True)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
