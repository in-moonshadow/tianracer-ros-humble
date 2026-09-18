#!/usr/bin/env python3
"""分析一轮 dwb_round 的采样，输出与基线同口径的对比指标。

核心指标（用于判定 sim_time 改动是否减少「锯齿式减速」）：
  1. 赛程窗口内的 vx 分布 —— 特别是 vx 卡在 min_vel_x(0.4) 的占比
  2. 减速段（vx<0.6 且持续>=0.3s）的段数与总时长
  3. 每圈的分段表现，以及「前方 1m 转弯量」分档后的低速占比

用法：python3 dwb_analyze.py <轮次目录>        # 目录内含 samples.csv, judge.log
      python3 dwb_analyze.py <samples.csv>     # 或直接给 csv
"""
import csv
import math
import os
import re
import sys

MIN_VEL = 0.4           # DWB min_vel_x，用于统计「卡在下限」的占比
SLOW_TH = 0.6           # 低速阈值
STEP = 0.05             # 弧长重采样步长（m）


def load(path):
    rows = []
    for r in csv.DictReader(open(path)):
        try:
            rows.append({'wall': float(r['wall']), 'sim': float(r['sim']),
                         'v': float(r['cmd_vx']), 'w': float(r['cmd_wz']),
                         'ov': float(r['odom_vx']),
                         'ox': float(r['odom_x']), 'oy': float(r['odom_y']),
                         'x': float(r['amcl_x']), 'y': float(r['amcl_y'])})
        except (ValueError, KeyError):
            continue
    rows.sort(key=lambda r: r['sim'])
    return rows


def race_window(rows, tail_move=0.3, tail_hold=5.0):
    """裁出赛程窗口：首个有效定位样本 → 终点。

    ⚠️ 尾部必须裁掉「完赛后停在原地」的静止段（实测基线有 45s，
    会让 vx<0.05 的占比从 0% 虚高到 28.7%）。判据：从末尾往前找，
    若连续 tail_hold 秒内位移 < tail_move 米，则截到该段起点。
    """
    ok = [r for r in rows if r['x'] == r['x'] and r['y'] == r['y']]
    if not ok:
        return []
    win = [r for r in rows if ok[0]['sim'] <= r['sim'] <= ok[-1]['sim']]
    # 从末尾回扫，剔除尾部静止段
    end = len(win)
    while end > 1:
        tail = win[:end]
        t_end = tail[-1]['sim']
        j = len(tail) - 1
        while j > 0 and t_end - tail[j]['sim'] < tail_hold:
            j -= 1
        seg = tail[j:]
        if len(seg) < 2:
            break
        moved = math.hypot(seg[-1]['x'] - seg[0]['x'], seg[-1]['y'] - seg[0]['y'])
        if moved < tail_move:
            end = j
        else:
            break
    return win[:end]


def resample(rows):
    """按弧长 0.05m 重采样（剔除 >0.5m 的定位跳变），用于稳健测航向变化。"""
    pts = [dict(rows[0], s=0.0)]
    acc = 0.0
    for a, b in zip(rows, rows[1:]):
        d = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        if d > 0.5 or d == 0:
            continue
        n = max(1, int(round(d / STEP)))
        for k in range(1, n + 1):
            t = k / n
            acc += d / n
            pts.append({'x': a['x'] + (b['x'] - a['x']) * t,
                        'y': a['y'] + (b['y'] - a['y']) * t,
                        'v': a['v'] + (b['v'] - a['v']) * t,
                        's': acc, 'sim': a['sim'] + (b['sim'] - a['sim']) * t})
    return pts


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def turn_ahead(pts, i, win=1.0):
    tot = 0.0
    k = i
    while k + 1 < len(pts) and pts[k]['s'] - pts[i]['s'] < win:
        h1 = math.atan2(pts[k + 1]['y'] - pts[k]['y'],
                        pts[k + 1]['x'] - pts[k]['x'])
        j0, j1 = max(0, k - 2), min(len(pts) - 1, k + 3)
        h2 = math.atan2(pts[j1]['y'] - pts[j0]['y'],
                        pts[j1]['x'] - pts[j0]['x'])
        tot += abs(wrap(h2 - h1))
        k += 1
    return tot


def slow_segments(rows, th=SLOW_TH, min_dur=0.3):
    segs, cur = [], None
    for r in rows:
        if r['v'] < th:
            if cur is None:
                cur = [r, r]
            else:
                cur[1] = r
        else:
            if cur is not None and cur[1]['sim'] - cur[0]['sim'] >= min_dur:
                segs.append(tuple(cur))
            cur = None
    if cur is not None and cur[1]['sim'] - cur[0]['sim'] >= min_dur:
        segs.append(tuple(cur))
    return segs


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(len(xs) * p))]


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else '/tmp/dwbround/r1'
    if os.path.isdir(arg):
        csv_path = os.path.join(arg, 'samples.csv')
        judge_path = os.path.join(arg, 'judge.log')
    else:
        csv_path = arg
        judge_path = os.path.join(os.path.dirname(arg), 'judge.log')

    if not os.path.exists(csv_path):
        print('找不到采样文件：%s' % csv_path)
        return 1

    rows = load(csv_path)
    if not rows:
        print('采样为空')
        return 1
    race = race_window(rows)
    if not race:
        print('赛程窗口为空（定位始终无效）')
        return 1

    print('=' * 62)
    print('轮次：%s' % os.path.basename(os.path.dirname(csv_path) or arg))
    print('总样本 %d，赛程窗口 %d（sim %.1f~%.1f s）'
          % (len(rows), len(race), race[0]['sim'], race[-1]['sim']))

    # ── 终局（若有 judge.log）──────────────────────────────────
    if os.path.exists(judge_path):
        try:
            txt = open(judge_path, encoding='utf-8', errors='replace').read()
            m = re.search(r'FINAL state: (\w+).*?score: ([\d.]+).*?points: (\d+)/(\d+)'
                          r'.*?move_dis: ([\d.]+)m.*?elapsed: ([\d.]+)s', txt, re.S)
            if m:
                print('终局：state=%s score=%s points=%s/%s 里程=%sm 用时=%ss'
                      % m.groups())
            else:
                tail = [l for l in txt.splitlines()
                        if 'FINAL' in l or 'has not moved' in l
                        or 'No checkpoint' in l]
                print('终局片段：%s' % (tail[-1][:110] if tail else '（无结束标志）'))
        except OSError:
            pass

    # ── 1) vx 分布 ─────────────────────────────────────────────
    vs = [r['v'] for r in race]
    n = len(vs)
    print('\n【1】赛程内 vx 分布（n=%d）' % n)
    print('  p10=%.3f p50=%.3f p90=%.3f 均值=%.3f'
          % (q(vs, .10), q(vs, .50), q(vs, .90), sum(vs) / n))
    for th in (0.05, 0.2, MIN_VEL + 1e-6, SLOW_TH, 1.0, 1.4):
        c = sum(1 for v in vs if v < th)
        print('  vx < %-5.2f : %5d = %5.1f%%' % (th, c, c / n * 100))
    stuck = sum(1 for v in vs if abs(v - MIN_VEL) < 0.005)
    print('  ★ 卡在 min_vel_x(%.1f)±0.005 : %d = %.1f%%'
          % (MIN_VEL, stuck, stuck / n * 100))

    # ── 2) 减速段 ──────────────────────────────────────────────
    segs = slow_segments(race)
    dur = sum(b['sim'] - a['sim'] for a, b in segs)
    span = race[-1]['sim'] - race[0]['sim']
    print('\n【2】减速段（vx<%.1f 且持续>=0.3s）' % SLOW_TH)
    print('  段数=%d  总时长=%.1fs / 赛程 %.1fs = %.1f%%'
          % (len(segs), dur, span, dur / span * 100))
    if segs:
        starts = [a['sim'] for a, _ in segs]
        gaps = [b - a for a, b in zip(starts, starts[1:])]
        if gaps:
            print('  相邻段间隔: p50=%.1fs  最小=%.1fs  最大=%.1fs'
                  % (q(gaps, .5), min(gaps), max(gaps)))

    # ── 3) 前方转弯量分档 ──────────────────────────────────────
    pts = resample(race)
    print('\n【3】前方 1.0m 累计转弯量 → 低速占比（n=%d，步长 %.2fm）'
          % (len(pts), STEP))
    print('      （<0.3=基本直线，>0.8=急弯；同档内比才公平）')
    for lo, hi in [(0, 0.3), (0.3, 0.8), (0.8, 9)]:
        sel = [pts[i]['v'] for i in range(len(pts))
               if lo <= turn_ahead(pts, i) < hi]
        if not sel:
            continue
        low = sum(1 for v in sel if v < SLOW_TH) / len(sel) * 100
        print('  转弯 %.2f~%.2f : n=%4d  vx p50=%.3f  vx<%.1f 占 %5.1f%%'
              % (lo, hi, len(sel), q(sel, .5), SLOW_TH, low))

    # ── 4) 指令 vs 实测（打滑检查）────────────────────────────
    ratios = [r['ov'] / r['v'] for r in race
              if r['v'] > 0.5 and r['ov'] == r['ov']]
    if ratios:
        print('\n【4】odom_vx / cmd_vx 比值（打滑检查）: p50=%.3f'
              % q(ratios, .5))
    return 0


if __name__ == '__main__':
    sys.exit(main())
