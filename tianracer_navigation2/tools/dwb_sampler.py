#!/usr/bin/env python3
"""DWB 赛程采样器：记录 cmd_vel / odom / amcl_pose / scan 滞后，供减速诊断使用。

输出列：wall,sim,cmd_vx,cmd_wz,odom_vx,odom_wz,odom_x,odom_y,amcl_x,amcl_y,
        scan_age,scan_stamp
用法：python3 dwb_sampler.py <输出csv>     # 由 dwb_round.sh 自动拉起
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/dwb_samples.csv'


class Sampler(Node):
    def __init__(self):
        super().__init__('dwb_sampler')
        self.cmd = (float('nan'), float('nan'))
        self.odom = (float('nan'),) * 4
        self.amcl = (float('nan'), float('nan'))
        self.scan_stamp = None
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_amcl, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.f = open(OUT, 'w')
        self.f.write('wall,sim,cmd_vx,cmd_wz,odom_vx,odom_wz,odom_x,odom_y,'
                     'amcl_x,amcl_y,scan_age,scan_stamp\n')
        self.create_timer(0.05, self.tick)      # 20 Hz

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z, p.x, p.y)

    def on_amcl(self, m):
        p = m.pose.pose.position
        self.amcl = (p.x, p.y)

    def on_scan(self, m):
        self.scan_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def tick(self):
        sim = self.get_clock().now().nanoseconds * 1e-9
        age = (sim - self.scan_stamp) if self.scan_stamp else float('nan')
        self.f.write('%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f\n' % (
            time.time(), sim, self.cmd[0], self.cmd[1], self.odom[0], self.odom[1],
            self.odom[2], self.odom[3], self.amcl[0], self.amcl[1], age,
            self.scan_stamp if self.scan_stamp else -1.0))
        self.f.flush()


def main():
    rclpy.init()
    n = Sampler()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
