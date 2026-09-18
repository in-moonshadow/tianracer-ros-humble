#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图配准采样器：把车依次**传送**到若干位姿，每个位姿记录一帧 (真值位姿, 激光)。

为「地图 ↔ 世界」刚体配准采集多位置样本。配套 tools/map_reg_fit.py 做联合拟合。

为什么必须用 /odom 而不是 AMCL 的 map->base_link
------------------------------------------------
AMCL 的职责就是「找一个让激光与地图最贴合的位姿」。若用 map->base_link 把激光点
投到地图系，得到的结果**必然与地图贴合**，配准出来恒为 δ≈0 —— 这是循环论证。
本机 /odom 由 gz 的 OdometryPublisher 系统插件发布（直接读模型世界位姿，**不是**
轮速积分），是无地图依赖的真值，正是配准需要的基准。

（早期「单点拟合」失败很可能就栽在这里：用贴合后的位姿去测贴合程度。）

为什么用传送而不是 Nav2 巡游
----------------------------
launch 已把 /world/default/set_pose 桥接成 ROS 服务（裁判 reset 用它）。传送：
  · 不受基准赛道 40% 窄道卡死率影响（巡游会中途卡住，采样覆盖不全）；
  · 单次约 1s，几十个位姿一分钟内采完；
  · 位姿可控、可复现。
采到的位姿**以 /odom 实测为准**（传送后车会被物理沉降/推挤，故不采信指令值）。

用法
----
  python3 map_reg_scan.py --map <地图.yaml> --out <输出目录> [--max-poses 60] [--seed 0]
前置：gz 仿真已在跑（tianracer_on_racetrack.launch.py），且 /scan、/odom、TF 正常。
产出：<输出目录>/scans.npz   —— poses(N,3)=[x,y,yaw] 真值, scans(N,K), angles(K), valid(N)
      <输出目录>/poses.json  —— 指令值 vs 实测值，便于核对传送是否奏效
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler

SETTLE_SEC = 0.8          # 传送后等待（仿真秒）：等物理沉降 + 扫描/TF 跟上
STABLE_EPS = 0.02         # 判定「已静止」的位移阈值（米）
GZ_MODEL_NAME = 'tianracer'
Z_SPAWN = 0.1


def load_map(path):
    """读 PGM + yaml，返回 (occ 布尔数组[H,W], res, origin)。行 0 = 图顶 = y 最大。"""
    with open(path) as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    img = os.path.join(os.path.dirname(os.path.abspath(path)), meta['image'])
    with open(img, 'rb') as f:
        data = f.read()
    # 解析 P5 头（magic, 可选注释, 宽, 高, 最大值）
    tok, i = [], 0
    while len(tok) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tok.append(data[i:j])
        i = j
    assert tok[0] == b'P5', '只支持 P5 PGM'
    W, H = int(tok[1]), int(tok[2])
    i += 1
    px = np.frombuffer(data, dtype=np.uint8, count=W * H, offset=i).reshape(H, W)
    occ = px < 100          # occupied_thresh 0.65 -> 值 <89 为占据
    oyaw = float(meta['origin'][2]) if len(meta['origin']) > 2 else 0.0
    return occ, res, (ox, oy, oyaw), px


def sample_poses(free, res, origin, max_poses, grid_m, clearance_m, seed):
    """在地图**已建图的自由格**里撒点：离墙够远、按 grid_m 稀释、上限 max_poses。

    注意 free 必须是真正的自由格（像素 254），**不能用 ~occ**：本图 88% 是 unknown(205)，
    用 ~occ 会把整片未建图区域当成自由空间，撒出的点大半落在赛道之外 —— 实测那批点
    激光回波为 0（周围超出量程），白白浪费一轮。
    """
    from scipy import ndimage
    # EDT(free)：对每个像素求「到最近的**非**自由格（墙/未知）」的距离，
    # 自由格上为正、非自由格上为 0。故 dist>=clearance 恰好选出「位于自由区且离墙够远」。
    # 写成 EDT(~free) 就反了 —— 那求的是到最近自由格的距离，会选出深处的未知区。
    dist = ndimage.distance_transform_edt(free) * res
    H, W = free.shape
    step = max(1, int(round(grid_m / res)))
    cand = []
    for j in range(step // 2, H, step):
        for i in range(step // 2, W, step):
            if dist[j, i] >= clearance_m:
                cand.append((i, j))
    if not cand:
        return []
    rng = np.random.default_rng(seed)
    rng.shuffle(cand)
    cand = cand[:max_poses]
    ox, oy = origin[0], origin[1]
    out = []
    for k, (i, j) in enumerate(cand):
        x = ox + res * (i + 0.5)
        y = oy + res * (H - j - 0.5)
        yaw = [(0.0), (math.pi / 2), (math.pi), (-math.pi / 2)][k % 4]
        out.append((x, y, yaw))
    return out


class Collector(Node):
    def __init__(self):
        super().__init__('map_reg_scan')
        self.scan = None
        self.odom = None            # (x, y, yaw, t_sim)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.cli = self.create_client(SetEntityPose, '/world/default/set_pose')
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.hist = []              # 近若干帧位姿，用于判静止
        self._t_tele = None         # 最近一次传送的仿真时刻

    def _on_scan(self, m):
        self.scan = m

    def _on_odom(self, m):
        p = m.pose.pose
        _, _, yaw = euler_from_quaternion([p.orientation.x, p.orientation.y,
                                           p.orientation.z, p.orientation.w])
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom = (p.position.x, p.position.y, yaw, t)
        self.hist.append((p.position.x, p.position.y))
        self.hist[:] = self.hist[-10:]

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def teleport(self, x, y, yaw):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return False, 'set_pose 服务不可用'
        req = SetEntityPose.Request()
        req.entity.name = GZ_MODEL_NAME
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(Z_SPAWN)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        req.pose.orientation.x, req.pose.orientation.y = q[0], q[1]
        req.pose.orientation.z, req.pose.orientation.w = q[2], q[3]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        ok = res is not None and res.success
        if ok:
            # 清掉传送前的扫描与位姿历史：否则 capture 的「已静止」判据会被
            # 旧位姿满足，采到的是传送前那一帧（静默错误，数据看着正常）。
            self._t_tele = self.odom[3] if self.odom else None
            self.scan = None
            self.hist = []
        return ok, ('ok' if ok else 'gz 返回失败')

    def capture(self):
        """等待静止后，返回 (laser_pose_in_odom(x,y,yaw), ranges, angles)。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < SETTLE_SEC * 5:
            self.spin_for(0.1)
            # 必须拿到传送之后产生的扫描，且位姿连续若干帧不动
            if self.scan is None or len(self.hist) < 5:
                continue
            stamp = self.scan.header.stamp.sec + self.scan.header.stamp.nanosec * 1e-9
            if self._t_tele is not None and stamp <= self._t_tele + 0.3:
                continue
            xs = np.array(self.hist[-5:])
            if np.max(np.abs(xs - xs[-1])) < STABLE_EPS:
                break
        # 传送后至少再等 SETTLE_SEC，保证物理沉降完、TF 也是新位姿下的
        self.spin_for(SETTLE_SEC)
        if self.scan is None:
            return None
        try:
            tr = self.tf.lookup_transform('odom', self.scan.header.frame_id,
                                          rclpy.time.Time())
        except Exception as e:
            return ('TF 查询失败: %s' % e)
        t = tr.transform.translation
        q = tr.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # 激光点：先旋到 odom 系，再平移
        ang = self.scan.angle_min + np.arange(len(self.scan.ranges)) * self.scan.angle_increment
        rng = np.asarray(self.scan.ranges, dtype=np.float64)
        return ((t.x, t.y, yaw), rng, ang)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--max-poses', type=int, default=60)
    ap.add_argument('--grid', type=float, default=1.0, help='采样网格边长(米)')
    # 0.35m 足够：传感器在车头前方 0.094m，只要它不在墙里即可，不必容纳整个车体。
    # 且赛道最窄处（0.98m 窄道）装不下 0.7m 的圆，clearance 给太大会把关键区域整个排除。
    ap.add_argument('--clearance', type=float, default=0.35, help='最小离墙距离(米)')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    occ, res, origin, px = load_map(a.map)
    free = (px >= 250)                       # 254 = 自由；205 = 未建图，必须排除
    print('地图 %d x %d @%.3fm，自由格 %d（%.1f%%）'
          % (px.shape[1], px.shape[0], res, int(free.sum()), 100 * free.mean()))
    poses = sample_poses(free, res, origin, a.max_poses, a.grid, a.clearance, a.seed)
    print('采样位姿 %d 个（grid=%.1fm clearance=%.1fm）' % (len(poses), a.grid, a.clearance))
    if not poses:
        print('!! 地图里找不到满足 clearance 的自由格', file=sys.stderr)
        return 2

    rclpy.init()
    node = Collector()
    print('等待 /scan 与 /odom …')
    t0 = time.monotonic()
    while (node.scan is None or node.odom is None) and time.monotonic() - t0 < 60:
        node.spin_for(0.2)
    if node.scan is None or node.odom is None:
        print('!! /scan 或 /odom 未就绪', file=sys.stderr)
        return 3
    print('就绪。首帧 odom=(%.3f, %.3f, %.3f)' % node.odom[:3])

    P, R, A, rec = [], [], [], []
    for k, (x, y, yaw) in enumerate(poses):
        ok, msg = node.teleport(x, y, yaw)
        if not ok:
            print('  [%2d/%d] 传送失败：%s' % (k + 1, len(poses), msg))
            continue
        cap = node.capture()
        if cap is None or isinstance(cap, str):
            print('  [%2d/%d] 采集失败：%s' % (k + 1, len(poses), cap or '无扫描'))
            continue
        (lx, ly, lyaw), rng, ang = cap
        good = np.isfinite(rng)
        P.append((lx, ly, lyaw))
        R.append(rng)
        A.append(ang)
        rec.append({'k': k, 'cmd': [x, y, yaw], 'odom': [lx, ly, lyaw],
                    'n_valid': int(good.sum()), 'n_total': int(len(rng))})
        print('  [%2d/%d] 指令(%.2f,%.2f,%.2f) 实测(%.2f,%.2f,%.2f) 有效点 %d/%d'
              % (k + 1, len(poses), x, y, yaw, lx, ly, lyaw, good.sum(), len(rng)))

    node.destroy_node()
    rclpy.shutdown()

    if not P:
        print('!! 一个样本都没采到', file=sys.stderr)
        return 4
    np.savez_compressed(os.path.join(a.out, 'scans.npz'),
                        poses=np.array(P), scans=np.array(R),
                        angles=np.array(A[0]), n_valid=np.array([r['n_valid'] for r in rec]))
    with open(os.path.join(a.out, 'poses.json'), 'w') as f:
        json.dump(rec, f, indent=1)
    print('已保存 %d 帧到 %s/scans.npz' % (len(P), a.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
