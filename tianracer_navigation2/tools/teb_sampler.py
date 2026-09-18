#!/usr/bin/env python3
"""TEB 赛程采样器：同时采平滑器前后的速度指令与里程计，用于判定
   ① 指令是否被 velocity_smoother 削  ② 打滑（指令 vs 实际推进）
   ③ 静止/爬行占比与角速度分布（判断控制器是否在"原地捻转"）
用法：python3 teb_sampler.py <输出csv>       # 由 teb_round.sh 自动拉起
输出列：t(墙钟),topic,vx,wz,x,y
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/teb_samples.csv'


class Sampler(Node):
    def __init__(self):
        super().__init__('tebdiag_sampler')
        self.fh = open(OUT, 'w', buffering=1)
        # 墙钟时间戳 + 话题 + vx + wz + x + y
        self.fh.write('t,topic,vx,wz,x,y\n')
        self.n = {'nav': 0, 'cmd': 0, 'odom': 0}
        self.create_subscription(Twist, '/cmd_vel_nav', self.cb_nav, 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 50)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 50)
        self.create_timer(5.0, self.report)

    def _w(self, topic, vx, wz, x='', y=''):
        self.fh.write('%.4f,%s,%.6f,%.6f,%s,%s\n' % (time.time(), topic, vx, wz, x, y))

    def cb_nav(self, m):
        self.n['nav'] += 1
        self._w('cmd_vel_nav', m.linear.x, m.angular.z)

    def cb_cmd(self, m):
        self.n['cmd'] += 1
        self._w('cmd_vel', m.linear.x, m.angular.z)

    def cb_odom(self, m):
        self.n['odom'] += 1
        self._w('odom', m.twist.twist.linear.x, m.twist.twist.angular.z,
                '%.6f' % m.pose.pose.position.x, '%.6f' % m.pose.pose.position.y)

    def report(self):
        self.get_logger().info('sampler counts: %s' % self.n)


def main():
    rclpy.init()
    node = Sampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
