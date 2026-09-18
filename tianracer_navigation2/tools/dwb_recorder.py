#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DWB 全程记录器：把「车在什么时候减速」与「为什么减速」分层记录下来。

三层设计（对应本文件的三类输出）：

  A. 状态层（20 Hz 全量）  →  <out>/state.csv
     位姿、AMCL、odom、平滑器前后指令、TF/scan 滞后。用来定位「什么时候减速」。

  B. 决策层（每次 /evaluation）→  <out>/eval.csv
     DWB 每个控制周期选中轨迹的速度，以及各 critic 的分项得分。
     用来回答「为什么减速」：是没有更快的可行轨迹（硬约束），
     还是有更快的但被打分压掉了（软代价）。

  C. 事件层（减速时存全量现场）→  <out>/events/<n>_<t>.json
     减速沿触发时，存该周期的**全部候选轨迹 + 分项得分 + 当时的全局/局部路径**。
     这样每次减速都有完整现场，不必事后推测。

时钟：同时记录 wall（墙钟 epoch）与 sim（仿真时间）。历史踩过坑——用仿真时间窗口
去筛墙钟时间戳会静默筛出空集，故两者都留、分析时先确认时钟类型。

用法：
  python3 dwb_recorder.py <输出目录>
  # 由 dwb_round.sh 自动拉起；也可单独运行：
  ros2 run tianracer_navigation2 dwb_recorder.py /tmp/rec
"""

import json
import math
import os
import sys
import time

import rclpy
import rclpy.time as rt
from dwb_msgs.msg import LocalPlanEvaluation
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener

# 减速判定阈值（与 dwb_analyze.py 保持同口径，便于交叉核对）
SLOW_TH = 0.6          # vx < 此值视为「低速」——语义阈值，state.csv 的 slowed 列用它

# state.csv 末尾 6 列记录的关节（判「卡住时车轮是否在转」「两前轮转向是否互相冲突」用）
# 舵角链：transform.py 把 δ 限幅 ±0.6，servo_commands.py 再把 δ 展开成左右轮各自的
# Ackermann 角（内侧轮转得更多）——δ 较大时内侧轮会超出关节极限 ±0.6，两前轮互相顶死。
JOINTS = {
    'left_steering_hinge_joint': 'lsteer_pos',
    'right_steering_hinge_joint': 'rsteer_pos',
    'left_front_wheel_joint': 'lfw_vel',
    'right_front_wheel_joint': 'rfw_vel',
    'left_rear_wheel_joint': 'lrw_vel',
    'right_rear_wheel_joint': 'rrw_vel',
}
SLOW_MIN_DUR = 0.3     # 低速持续超过此秒数才算一段减速
EVENT_COOLDOWN = 2.0   # 现场之间最小间隔（避免刷爆磁盘）

# C 层「存全候选现场」的触发阈值。
# ⚠️ 它必须与 SLOW_TH 分开。原先现场触发写的是 `vx < SLOW_TH and self._slowed`，
#    这在 min_vel_x=0.4 时代成立；后来 min_vel_x 抬到 0.6，而 SLOW_TH 仍是 0.6，
#    于是 `vx < 0.6` 恒不成立 ⇒ **events/ 再也没有落过盘**（2026-09-15 实测确认）。
#    又不能直接把 SLOW_TH 抬高：它同时喂给 state.csv 的 slowed 列与 dwb_analyze.py，
#    改了会让历史轮次的「减速段」口径失去可比性。
#    故另立此阈值，只控制「哪些帧值得存下全部 420 条候选」。
#    取 2.0：racetrack_1 上选中 vx<2.0 的帧约占 4~8%，配合 2s 冷却每轮约 30~70 次、
#    每次 ~25KB，合计 ~2MB，可控。
EVENT_VX_TH = 2.0
# ── 诊断用：高速触发（2026-09-18，提速诊断；永久保留）────────────────────
# 动因：原触发只有「vx < 2.0」，于是落盘的全是**弯中减速帧**——这是一个真实的
#   **观测盲区**，且已经害过一次：在弯中帧上做 PathAlign 反事实，得出「降权重完全
#   无效」的错误结论（换到直道帧上结论相反）。巡航/跨格行为必须能看到。
# 取 2.55：落在 DWB 候选栅格点 2.495 与上一格 2.621 之间，捕获「成功跨格」的时刻。
# ⚠️ 抬 EVENT_VX_TH 是**反方向**（只会把 2.49 主峰也纳进来、仍抓不到跨格帧）。
# ⚠️ 生效条件：本分支只在 `max_vel_x > 2.55` 时可能触发。r1 的 max_vel_x 已于
#   2026-09-18 回退到 2.5（栅格顶点即 2.4947）⇒ **在 r1 上此分支恒不触发**，
#   仅当再次抬速时复活。test_indoor(1.5) 等其他赛道同样不触发，无副作用。
EVENT_FAST_VX_TH = 2.55

STATE_DT = 0.05        # 状态层采样周期（20 Hz）


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DwbRecorder(Node):
    def __init__(self, outdir):
        super().__init__('dwb_recorder')
        self.out = outdir
        os.makedirs(outdir, exist_ok=True)
        os.makedirs(os.path.join(outdir, 'events'), exist_ok=True)

        # 最新消息缓存（回调只存，采样时统一写盘，避免高频 IO）
        self.cmd_nav = (float('nan'), float('nan'))   # 平滑器前
        self.cmd = (float('nan'), float('nan'))       # 平滑器后
        self.odom = (float('nan'),) * 4               # vx, wz, x, y
        self.amcl = (float('nan'), float('nan'))
        self.scan_stamp = None
        self.plan = []                                # 全局路径点
        self.local_plan = []
        self.tf_age = float('nan')                    # map→base_footprint 的 TF 年龄
        # 关节状态：(舵角位置, 车轮角速度)；未收到时为 nan
        self.joint = {j: (float('nan'), float('nan')) for j in JOINTS}

        self.state_f = open(os.path.join(outdir, 'state.csv'), 'w', buffering=1)
        self.state_f.write('wall,sim,vx_nav,wz_nav,vx_cmd,wz_cmd,'
                           'odom_vx,odom_wz,odom_x,odom_y,amcl_x,amcl_y,'
                           'scan_age,tf_age,slowed,'
                           + ','.join(JOINTS.values()) + '\n')

        self.eval_f = open(os.path.join(outdir, 'eval.csv'), 'w', buffering=1)
        self.eval_f.write('wall,sim,n_twists,best_idx,best_vx,best_wz,best_total,'
                          'worst_idx,worst_total,' + ','.join(
                              'best_%s' % c for c in (
                                  'Oscillation', 'BaseObstacle', 'PathAlign',
                                  'PathDist', 'GoalDist')) + '\n')

        self.n_state = 0
        self.n_eval = 0
        self.n_event = 0
        self._slowed = False          # 当前是否处于低速段
        self._slow_since = None
        self._last_event_t = -1e9
        self._last_fast_t = -1e9

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        # 订阅
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav, 20)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_amcl, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Path, '/local_plan', self.on_local_plan, 10)
        self.create_subscription(LocalPlanEvaluation, '/evaluation',
                                 self.on_eval, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint, 20)

        self.create_timer(STATE_DT, self.tick)
        self.get_logger().info('recorder ready -> %s' % outdir)

    # ── 回调 ────────────────────────────────────────────────────
    def on_nav(self, m):
        self.cmd_nav = (m.linear.x, m.angular.z)

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

    def on_joint(self, m):
        # velocity 可能为空数组（某些 broadcaster 只发 position）——长度不符时按 nan 记，
        # 不能直接 zip(m.name, m.position, m.velocity)（会静默什么都不记）。
        vel = m.velocity if len(m.velocity) == len(m.name) else [float('nan')] * len(m.name)
        for name, pos, v in zip(m.name, m.position, vel):
            if name in self.joint:
                self.joint[name] = (pos, v)

    def on_plan(self, m):
        self.plan = [(ps.pose.position.x, ps.pose.position.y)
                     for ps in m.poses]

    def on_local_plan(self, m):
        self.local_plan = [(ps.pose.position.x, ps.pose.position.y)
                           for ps in m.poses]

    def on_eval(self, m):
        """决策层：记录胜出轨迹的速度与各 critic 分项得分。

        ⚠️ 性能：/evaluation 是 18Hz × 400 条候选（约 798KB/条）。故常态路径**只读
        best/worst 两条**，绝不遍历全部候选——那 400 次 dict 构建只在减速存现场时做，
        否则会白白烧掉 CPU（本节点与 Nav2 同机运行，须保持轻量）。
        """
        twists = m.twists
        if not twists:
            return
        bi = m.best_index
        if bi >= len(twists):
            return
        best = twists[bi]
        wi = m.worst_index if m.worst_index < len(twists) else bi
        worst = twists[wi]

        scores = {s.name: s.raw_score for s in best.scores}
        C = ('Oscillation', 'BaseObstacle', 'PathAlign', 'PathDist', 'GoalDist')

        self.eval_f.write('%s\n' % ','.join(str(v) for v in (
            '%.4f' % time.time(),
            '%.4f' % (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
            len(twists), bi,
            '%.6f' % best.traj.velocity.x,
            '%.6f' % best.traj.velocity.theta,
            '%.4f' % best.total,
            wi, '%.4f' % worst.total,
            *['%.4f' % scores.get(c, float('nan')) for c in C])))
        self.n_eval += 1

        # C 层：存全量现场（此处才遍历全部候选）
        # 触发用 EVENT_VX_TH 而非 SLOW_TH；不再要求 self._slowed——弯道里的减速段
        # 未必经过「vx<0.6 且持续 0.3s」，用 SLOW_TH 门控会让现场几乎永远不落盘。
        vx = best.traj.velocity.x
        # 两条触发线各自独立冷却：否则弯中减速帧会把「跨格帧」的名额占光。
        if vx == vx and vx < EVENT_VX_TH:
            now = time.time()
            if now - self._last_event_t > EVENT_COOLDOWN:
                self._last_event_t = now
                self._dump_event(m, vx, bi, scores)
        elif vx == vx and vx >= EVENT_FAST_VX_TH:
            now = time.time()
            if now - self._last_fast_t > EVENT_COOLDOWN:
                self._last_fast_t = now
                self._dump_event(m, vx, bi, scores)

    # ── 采样与写盘 ──────────────────────────────────────────────
    def tick(self):
        sim = self.get_clock().now().nanoseconds * 1e-9
        scan_age = (sim - self.scan_stamp) if self.scan_stamp else float('nan')

        # TF 年龄：map → base_footprint（用缓存，不阻塞）
        tf_age = float('nan')
        try:
            tr = self._tf.lookup_transform('map', 'base_footprint', rt.Time())
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            tf_age = sim - stamp
        except Exception:
            pass

        # 减速状态机（供 C 层触发）
        vx = self.cmd[0]
        if vx == vx and vx < SLOW_TH:
            if not self._slowed:
                self._slowed = True
                self._slow_since = sim
        else:
            self._slowed = False
            self._slow_since = None

        self.state_f.write('%s\n' % ','.join(str(v) for v in (
            '%.4f' % time.time(), '%.4f' % sim,
            '%.4f' % self.cmd_nav[0], '%.4f' % self.cmd_nav[1],
            '%.4f' % self.cmd[0], '%.4f' % self.cmd[1],
            '%.4f' % self.odom[0], '%.4f' % self.odom[1],
            '%.4f' % self.odom[2], '%.4f' % self.odom[3],
            '%.4f' % self.amcl[0], '%.4f' % self.amcl[1],
            '%.4f' % scan_age, '%.4f' % tf_age,
            1 if self._slowed else 0,
            '%.4f' % self.joint['left_steering_hinge_joint'][0],
            '%.4f' % self.joint['right_steering_hinge_joint'][0],
            '%.4f' % self.joint['left_front_wheel_joint'][1],
            '%.4f' % self.joint['right_front_wheel_joint'][1],
            '%.4f' % self.joint['left_rear_wheel_joint'][1],
            '%.4f' % self.joint['right_rear_wheel_joint'][1])))
        self.n_state += 1

    def _dump_event(self, m, vx, bi, scores):
        """C 层：存这次减速的完整现场。

        一条 /evaluation 约 400 条候选、798KB（文本）——全量记录不可行（18Hz 会到
        840MB/分钟）。故此处只保留**决策所需的数值**：每条候选的速度、总分与各 critic
        分项得分（约 25KB/次），足够回答「为什么减速」：
          - 若存在更快的候选且总分有限 → 是软代价权重压掉了它
          - 若更快的候选总分全为 inf → 是硬约束（撞格），当时确实没有可行快解
        """
        self.n_event += 1
        sim = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # 文件名带触发来源：L=低速线(vx<EVENT_VX_TH) / F=高速线(vx>=EVENT_FAST_VX_TH)
        tag = 'F' if vx >= EVENT_FAST_VX_TH else 'L'
        fn = os.path.join(self.out, 'events',
                          '%03d_%s_t%.2f.json' % (self.n_event, tag, sim))

        inf = float('inf')
        cands = []
        scales = {}
        for t in m.twists:
            sc = {}
            sc_scale = {}
            for s in t.scores:
                sc[s.name] = s.raw_score
                sc_scale[s.name] = s.scale
                # scale 是全局常量，记一次即可（用于把 raw 还原成实际贡献）
                scales.setdefault(s.name, s.scale)
            cands.append({
                'vx': t.traj.velocity.x,
                'wz': t.traj.velocity.theta,
                'total': t.total if math.isfinite(t.total) else None,
                'critics': {k: (v if math.isfinite(v) else None)
                            for k, v in sc.items()},
            })
        # 按 vx 降序：分析时可直接看「是否存在更快的可行候选」
        cands.sort(key=lambda c: -c['vx'])
        # ⚠️ total < 0 是 DWB 的「非法轨迹」哨兵（BaseObstacle == -1 表示撞致命格），
        # 不是可行解。早先按 `is not None` 判定可行，把撞墙轨迹算成了可行，
        # 导致「最快可行 vx」偏高。这里一并修正。
        feasible = [c for c in cands
                    if c['total'] is not None and c['total'] >= 0]
        fastest_feasible = feasible[0] if feasible else None

        data = {
            'sim': sim, 'wall': time.time(),
            'chosen_vx': vx, 'chosen_wz': m.twists[bi].traj.velocity.theta,
            'best_index': bi, 'n_twists': len(m.twists),
            'n_feasible': len(feasible),
            'n_infeasible': len(cands) - len(feasible),
            'fastest_feasible_vx': (fastest_feasible['vx']
                                    if fastest_feasible else None),
            'critic_scales': scales,     # 消息自带的权威 scale（非 yaml）
            'critic_scores_of_best': scores,
            'pose': {'x': self.odom[2], 'y': self.odom[3],
                     'amcl_x': self.amcl[0], 'amcl_y': self.amcl[1]},
            'cmd_nav': list(self.cmd_nav), 'cmd': list(self.cmd),
            'global_plan': self.plan[:200],
            'local_plan': self.local_plan[:200],
            'candidates_by_vx_desc': cands,
        }
        try:
            with open(fn, 'w') as f:
                json.dump(data, f, indent=1)
        except OSError as e:
            self.get_logger().warn('存现场失败: %s' % e)

    def report(self):
        self.get_logger().info(
            'recorded: state=%d eval=%d events=%d' % (
                self.n_state, self.n_eval, self.n_event))

    def close(self):
        self.state_f.close()
        self.eval_f.close()


def main():
    rclpy.init()
    outdir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/dwb_rec'
    node = DwbRecorder(outdir)
    node.create_timer(10.0, node.report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
