#!/usr/bin/env python3
"""定位分析：轮次目录 -> 逐 5s 曲线 + vx≈0 的位置分布 + 异常计数（统一用采样器墙钟 t 列）。"""
import csv
import math
import os
import re
import sys
from collections import Counter


def load(d):
    nav, odom = [], []
    with open(os.path.join(d, 'samples.csv'), errors='replace') as fh:
        r = csv.reader(fh)
        next(r, None)
        for p in r:
            if len(p) < 6:
                continue
            try:
                t, vx, wz = float(p[0]), float(p[2]), float(p[3])
            except ValueError:
                continue
            x = float(p[4]) if p[4] else None
            y = float(p[5]) if p[5] else None
            if p[1] == 'cmd_vel_nav':
                nav.append((t, vx, wz))
            elif p[1] == 'odom' and x is not None:
                odom.append((t, x, y))
    nav.sort(); odom.sort()
    return nav, odom


def pct(v, q):
    if not v:
        return float('nan')
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))]


def main(d):
    t0 = float(open(os.path.join(d, 't_start.txt')).read().strip())
    nav, odom = load(d)
    nav = [s for s in nav if s[0] >= t0]
    odom = [s for s in odom if s[0] >= t0]
    tmax = max([s[0] for s in nav] or [t0])
    dur = tmax - t0
    print('窗口 %.1fs  nav样本 %d  odom样本 %d' % (dur, len(nav), len(odom)))

    vxs = [abs(s[1]) for s in nav]
    wzs = [abs(s[2]) for s in nav]
    stall = [s for s in nav if abs(s[1]) < 0.05]
    print('vx:   p50=%.3f p90=%.3f max=%.3f' % (pct(vxs, .5), pct(vxs, .9), max(vxs)))
    print('|wz|: p50=%.3f p90=%.3f max=%.3f' % (pct(wzs, .5), pct(wzs, .9), max(wzs)))
    print('静止占比(|vx|<0.05): %.1f%% (%d/%d)' % (100.0 * len(stall) / max(1, len(nav)), len(stall), len(nav)))

    # 里程与逐 5s
    dist = 0.0
    for a, b in zip(odom, odom[1:]):
        dist += math.hypot(b[1] - a[1], b[2] - a[2])
    moving_t = dur * (1 - len(stall) / max(1, len(nav)))
    print('位置差分总里程 %.2f m ；运动段均速 %.3f m/s' % (dist, dist / moving_t if moving_t > 0 else 0))

    print('逐5s(里程m | 均速 | 均值vx | 均值|wz| | 段末位置):')
    b = t0
    while b < tmax:
        seg = [s for s in nav if b <= s[0] < b + 5]
        seg_o = [s for s in odom if b <= s[0] < b + 5]
        mv = 0.0
        for a_, c_ in zip(seg_o, seg_o[1:]):
            mv += math.hypot(c_[1] - a_[1], c_[2] - a_[2])
        if seg:
            pos = '(%6.2f,%6.2f)' % (seg_o[-1][1], seg_o[-1][2]) if seg_o else '     -     '
            print('  t=%4.0fs  %5.2f  %5.2f  %5.3f  %5.3f  %s' %
                  (b - t0, mv, mv / 5.0,
                   sum(abs(s[1]) for s in seg) / len(seg),
                   sum(abs(s[2]) for s in seg) / len(seg), pos))
        b += 5

    # vx≈0 时段的位置分布（1m 网格）+ 与首个目标的距离
    goals = [(-2.98, 0.13)]
    grid = Counter()
    near_goal = 0
    stall_pos = []
    for s in stall:
        t = s[0]
        o = [p for p in odom if abs(p[0] - t) < 0.15]
        if not o:
            continue
        x, y = o[0][1], o[0][2]
        stall_pos.append((x, y))
        grid[(round(x), round(y))] += 1
        if math.hypot(x - goals[0][0], y - goals[0][1]) < 1.5:
            near_goal += 1
    print('vx≈0 样本中可取到位置的: %d' % len(stall_pos))
    if stall_pos:
        print('  位置分布(1m 网格, top6): ' + '  '.join('%s:%d' % (k, v) for k, v in grid.most_common(6)))
        dg = [math.hypot(x - goals[0][0], y - goals[0][1]) for x, y in stall_pos]
        print('  距首目标(-2.98,0.13) 距离: p50=%.2fm  最小=%.2fm  在1.5m内占比=%.1f%%' %
              (pct(dg, .5), min(dg), 100.0 * near_goal / len(stall_pos)))

    # 异常计数（nav2 日志，计数与时钟无关）
    pats = ['trajectory is not feasible', 'Controller patience exceeded',
            'No valid trajectories', 'failed to create plan',
            'max_vel_x_backwards', 'possible oscillation']
    txt = open(os.path.join(d, 'nav2.log'), errors='replace').read()
    print('异常计数: ' + '; '.join('%s=%d' % (p, txt.count(p)) for p in pats))


if __name__ == '__main__':
    for d in sys.argv[1:]:
        print('########## %s ##########' % d)
        main(d)
        print()
