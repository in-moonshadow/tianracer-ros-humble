#!/usr/bin/env python3
"""TEB 最后一轮调优验证分析。

输入：一个轮次目录（含 judge.log / nav2.log / samples.csv / t_start.txt）
输出：
  1) 门间隔（直接取裁判日志的「段距离/段用时/段速」）
  2) 终局与触发判据
  3) 逐 5s 里程曲线 + 静止占比 + 运动段均速
  4) 死法归类：not feasible 的时间分布（bucketed）vs ω/vx 的 p50
"""
import os
import re
import sys
from collections import defaultdict

TS = re.compile(r'\[(\d{10}\.\d+)\]')


def stats(xs):
    if not xs:
        return None
    xs = sorted(xs)
    n = len(xs)

    def p(q):
        return xs[min(n - 1, int(q * n))]
    return dict(n=n, max=xs[-1], p50=p(0.5), p90=p(0.9))


def fmt(d):
    if not d:
        return 'n=0'
    return 'n=%d max=%.3f p50=%.3f p90=%.3f' % (d['n'], d['max'], d['p50'], d['p90'])


def parse_judge(path):
    gates, final, crit = [], [], []
    if not os.path.exists(path):
        return gates, final, crit
    with open(path, errors='replace') as fh:
        for ln in fh:
            if '通过第' in ln:
                d = re.search(r'段距离=([\d.]+)m', ln)
                t = re.search(r'段用时=([\d.]+)s', ln)
                v = re.search(r'段速=([\d.]+)m/s', ln)
                gates.append((float(d.group(1)) if d else None,
                              float(t.group(1)) if t else None,
                              float(v.group(1)) if v else None, ln.strip()[:90]))
            if 'FINAL state' in ln:
                final.append(ln.strip())
            if 'has not moved' in ln or 'No checkpoint detected' in ln:
                crit.append(ln.strip()[:110])
    return gates, final, crit


def parse_nav2_events(path, t0):
    """返回 {'not feasible': [...相对秒], 'patience': [...]}"""
    ev = defaultdict(list)
    if not os.path.exists(path):
        return ev
    pats = {'not feasible': 'trajectory is not feasible',
            'patience': 'Controller patience exceeded',
            'oscillation': 'possible oscillation',
            'backwards_warn': 'max_vel_x_backwards'}
    with open(path, errors='replace') as fh:
        for ln in fh:
            m = TS.search(ln)
            if not m:
                continue
            t = float(m.group(1))
            for k, p in pats.items():
                if p in ln:
                    ev[k].append(t - t0)
                    break
    return ev


def parse_samples(path):
    """长格式 t,topic,vx,wz,x,y（t 为墙钟 epoch 秒）"""
    out = defaultdict(list)
    if not os.path.exists(path):
        return out
    with open(path, errors='replace') as fh:
        next(fh, None)
        for ln in fh:
            p = ln.strip().split(',')
            if len(p) < 4:
                continue
            try:
                t, vx, wz = float(p[0]), float(p[2]), float(p[3])
            except ValueError:
                continue
            x = float(p[4]) if len(p) > 4 and p[4] not in ('', 'nan') else None
            y = float(p[5]) if len(p) > 5 and p[5] not in ('', 'nan') else None
            out[p[1]].append((t, vx, wz, x, y))
    for k in out:
        out[k].sort()
    return out


def mileage_curve(odom, t_start, t_end, bucket=5.0):
    """按 5s 分桶的里程（用 odom 位置差分累计）"""
    rows = [(t, x, y) for (t, _, _, x, y) in odom if x is not None and y is not None]
    if len(rows) < 2:
        return []
    out, i = [], 0
    b = t_start
    while b < t_end:
        seg = [(t, x, y) for (t, x, y) in rows if b <= t < b + bucket]
        dist = 0.0
        for a, c in zip(seg, seg[1:]):
            dist += ((c[1] - a[1]) ** 2 + (c[2] - a[2]) ** 2) ** 0.5
        if seg:
            out.append((b - t_start, dist, dist / bucket))
        b += bucket
    return out


def main():
    d = sys.argv[1].rstrip('/')
    t0 = None
    p = os.path.join(d, 't_start.txt')
    if os.path.exists(p):
        t0 = float(open(p).read().strip())

    gates, final, crit = parse_judge(os.path.join(d, 'judge.log'))
    print('=== 门间隔 ===')
    if gates:
        for i, (dist, tt, v, raw) in enumerate(gates, 1):
            print('  门%-2d 段距离=%-7s 段用时=%-8s 段速=%-6s' %
                  (i, dist, tt, v))
        ints = [g[1] for g in gates if g[1] is not None]
        if ints:
            print('  间隔列表: %s' % ' / '.join('%.2f' % x for x in ints))
            print('  最大间隔: %.2f s  (是否全部 <30s: %s)' % (max(ints), max(ints) < 30))
    else:
        print('  （无门通过记录）')

    print('=== 终局 ===')
    for ln in final:
        print('  ' + ln)
    print('=== 触发判据 ===')
    for ln in crit:
        print('  ' + ln)
    if not crit:
        print('  （无终止判据命中）')

    # events
    ev = parse_nav2_events(os.path.join(d, 'nav2.log'), t0 or 0.0)
    print('=== 事件计数/时间分布 ===')
    for k in ('not feasible', 'patience', 'oscillation', 'backwards_warn'):
        v = ev.get(k, [])
        print('  %-14s n=%d' % (k, len(v)))
        if v and k in ('not feasible', 'patience'):
            bk = defaultdict(int)
            for t in v:
                bk[int(t // 5) * 5] += 1
            print('     每5s: ' + '  '.join('%d:%d' % (a, b) for a, b in sorted(bk.items())))

    # samples
    sm = parse_samples(os.path.join(d, 'samples.csv'))
    if not sm or t0 is None:
        print('=== 采样 ===\n  （无数据）')
        return
    t_end = t0 + 60.0
    pe = os.path.join(d, 't_end.txt')
    if os.path.exists(pe):
        t_end = float(open(pe).read().strip())

    odom = sm.get('odom', [])
    nav = [(t, vx, wz) for (t, vx, wz, _, _) in sm.get('cmd_vel_nav', [])]
    cmd = [(t, vx, wz) for (t, vx, wz, _, _) in sm.get('cmd_vel', [])]

    win = [(t, vx, wz, x, y) for (t, vx, wz, x, y) in odom if t0 <= t <= t_end]
    print('=== 赛程窗口（采样）===')
    print('  窗口长度 %.1f s，odom 样本 %d' % (t_end - t0, len(win)))

    curve = mileage_curve(odom, t0, t_end)
    if curve:
        tot = sum(c[1] for c in curve)
        print('  逐5s里程: ' + '  '.join('%.2f' % c[1] for c in curve))
        print('  逐5s均速: ' + '  '.join('%.2f' % c[2] for c in curve))
        print('  窗口总里程(位置差分) %.2f m' % tot)

    # 静止占比 & 运动段均速（用 cmd_vel_nav 的 vx，与既有口径一致）
    nv = [(t, vx, wz) for (t, vx, wz) in nav if t0 <= t <= t_end]
    if nv:
        stall = [x for x in nv if abs(x[1]) < 0.05]
        print('  前级 vx: %s' % fmt(stats([abs(x[1]) for x in nv])))
        print('  前级 wz: %s' % fmt(stats([abs(x[2]) for x in nv])))
        print('  静止占比(|vx|<0.05): %.1f%% (%d/%d)' %
              (100.0 * len(stall) / len(nv), len(stall), len(nv)))
        if curve:
            tot = sum(c[1] for c in curve)
            moving = (t_end - t0) * (1 - len(stall) / len(nv))
            if moving > 0:
                print('  运动段均速(位置差分/运动时长): %.3f m/s' % (tot / moving))

    cv = [(t, vx, wz) for (t, vx, wz) in cmd if t0 <= t <= t_end]
    if cv:
        print('  后级 vx: %s' % fmt(stats([abs(x[1]) for x in cv])))
        print('  后级 wz: %s' % fmt(stats([abs(x[2]) for x in cv])))


if __name__ == '__main__':
    for p in sys.argv[1:]:
        print('########## %s ##########' % p)
        main_p = p
        sys.argv = [sys.argv[0], main_p]
        main()
        print()
