#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析 dwb_recorder.py 的三层记录，输出「什么时候减速 + 为什么减速」的报告。

用法：
  python3 dwb_rec_report.py <rec 目录>          # 目录含 state.csv / eval.csv / events/
  python3 dwb_rec_report.py <rec 目录> --events # 额外逐条列出减速现场的关键判据

核心问题与对应判据：
  ① 什么时候减速？ → state.csv 的 vx_cmd 时序 + slow 段
  ② 是规划器主动慢，还是平滑器/硬件削的？ → vx_nav(平滑器前) vs vx_cmd(平滑器后)
  ③ 为什么减速？ → events/*.json 的 fastest_feasible_vx vs chosen_vx
       - fastest_feasible_vx 明显更大 → 存在更快的可行解却被选中 → **软代价（权重）问题**
       - fastest_feasible_vx ≈ chosen_vx   → 当时确实没有更快可行解 → **硬约束（几何/障碍）**
       - n_feasible 很小/为 0              → 候选空间被硬约束吃光
"""

import csv
import json
import math
import os
import sys


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(len(xs) * p))] if xs else float('nan')


def load_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(d, k, dflt=float('nan')):
    try:
        return float(d[k])
    except (KeyError, ValueError, TypeError):
        return dflt


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    rec = sys.argv[1]
    show_events = '--events' in sys.argv
    state = load_csv(os.path.join(rec, 'state.csv'))
    evals = load_csv(os.path.join(rec, 'eval.csv'))
    evdir = os.path.join(rec, 'events')
    events = sorted(f for f in os.listdir(evdir) if f.endswith('.json')) \
        if os.path.isdir(evdir) else []

    print('=' * 66)
    print('记录目录: %s' % rec)
    print('state 行数=%d  eval 行数=%d  减速现场=%d 个'
          % (len(state), len(evals), len(events)))
    if not state:
        print('\n⚠️ state.csv 为空。检查记录器是否在 /start 前启动、话题是否有数据。')
        return 1

    # 有效区段：AMCL 有定位 且 odom 非零
    valid = [r for r in state
             if fnum(r, 'amcl_x') == fnum(r, 'amcl_x')
             and fnum(r, 'odom_x') == fnum(r, 'odom_x')]
    print('有效状态行=%d' % len(valid))

    # ── ① 什么时候减速 ────────────────────────────────────────
    vs = [fnum(r, 'vx_cmd') for r in valid if fnum(r, 'vx_cmd') == fnum(r, 'vx_cmd')]
    print('\n【① 什么时候减速】vx_cmd（实际下发）分布，n=%d' % len(vs))
    print('  p10=%.3f p50=%.3f p90=%.3f 均值=%.3f'
          % (q(vs, .10), q(vs, .50), q(vs, .90), sum(vs) / len(vs)))
    SLOW = 0.6
    for th in (0.1, 0.4, SLOW, 1.0):
        c = sum(1 for v in vs if v < th)
        print('  vx < %-4.1f : %5d = %5.1f%%' % (th, c, c / len(vs) * 100))

    # 减速段
    segs, cur = [], None
    for r in valid:
        v = fnum(r, 'vx_cmd')
        t = fnum(r, 'sim')
        if v == v and v < SLOW:
            if cur is None:
                cur = [t, t, r]
            else:
                cur[1] = t
        else:
            if cur is not None and cur[1] - cur[0] >= 0.3:
                segs.append(cur)
            cur = None
    if cur is not None and cur[1] - cur[0] >= 0.3:
        segs.append(cur)
    dur = sum(s[1] - s[0] for s in segs)
    span = valid[-1]['sim'] and (fnum(valid[-1], 'sim') - fnum(valid[0], 'sim'))
    print('\n  减速段（vx<%.1f 且 >=0.3s）: %d 段, 合计 %.1fs%s'
          % (SLOW, len(segs), dur, (' / 全程 %.1fs = %.1f%%'
             % (span, dur / span * 100)) if span else ''))
    print('\n  %-4s %-9s %-8s %-8s %-26s' % ('#', '起(s)', '时长', 'minvx', '位置(odom)'))
    for i, (t0, t1, r) in enumerate(segs, 1):
        print('  %-4d %-9.1f %-8.1f %-8.2f (%7.2f,%7.2f)'
              % (i, t0, t1 - t0, min(
                  fnum(x, 'vx_cmd') for x in valid
                  if t0 <= fnum(x, 'sim') <= t1), fnum(r, 'odom_x'), fnum(r, 'odom_y')))

    # ── ② 是规划器主动慢，还是被削的 ──────────────────────────
    pairs = [(fnum(r, 'vx_nav'), fnum(r, 'vx_cmd')) for r in valid
             if fnum(r, 'vx_nav') == fnum(r, 'vx_nav')
             and fnum(r, 'vx_cmd') == fnum(r, 'vx_cmd')]
    if pairs:
        cut = [(a, b) for a, b in pairs if a - b > 0.05]
        print('\n【② 平滑器是否削了速度】vx_nav(前) vs vx_cmd(后)')
        print('  样本=%d  被削(a-b>0.05)=%d = %.1f%%'
              % (len(pairs), len(cut), len(cut) / len(pairs) * 100))
        if cut:
            print('  削幅 p50=%.3f max=%.3f'
                  % (q([a - b for a, b in cut], .5), max(a - b for a, b in cut)))
            print('  → 若这部分占比高，减速主因在**平滑器**；否则在**规划器决策**。')
        # 打滑：odom_vx / cmd
        slip = [(fnum(r, 'odom_vx'), fnum(r, 'vx_cmd')) for r in valid
                if fnum(r, 'vx_cmd') > 0.5]
        if slip:
            rr = [o / c for o, c in slip if c > 0]
            print('  odom_vx/vx_cmd 比值 p50=%.3f（<1 表示打滑）' % q(rr, .5))

    # ── ③ 为什么减速：决策层 ──────────────────────────────────
    if evals:
        bv = [fnum(e, 'best_vx') for e in evals if fnum(e, 'best_vx') == fnum(e, 'best_vx')]
        print('\n【③ 规划器选了什么速度】best_vx 分布，n=%d' % len(bv))
        print('  p10=%.3f p50=%.3f p90=%.3f' % (q(bv, .10), q(bv, .50), q(bv, .90)))
        slow_e = [e for e in evals if fnum(e, 'best_vx') < SLOW]
        print('  best_vx < %.1f 的周期: %d = %.1f%%'
              % (SLOW, len(slow_e), len(slow_e) / len(evals) * 100))
        print('\n  critic 分项得分（低速周期 vs 全部周期的中位数）:')
        for c in ('Oscillation', 'BaseObstacle', 'PathAlign', 'PathDist', 'GoalDist'):
            k = 'best_%s' % c
            allv = [fnum(e, k) for e in evals if fnum(e, k) == fnum(e, k)]
            slowv = [fnum(e, k) for e in slow_e if fnum(e, k) == fnum(e, k)]
            if allv:
                print('    %-14s 全部 p50=%-10.3f 低速 p50=%-10.3f'
                      % (c, q(allv, .5), q(slowv, .5) if slowv else float('nan')))

    # ── ④ 减速现场的判据（最关键）────────────────────────────
    print('\n【④ 为什么减速——每次现场的判据】')
    if not events:
        print('  无现场数据（本次未触发减速，或记录器未起）')
    else:
        print('  %-22s %-8s %-8s %-9s %-9s %s'
              % ('现场文件', '实选vx', '最快可行', '可行候选', '总候选', '判定'))
        verdicts = {'soft': 0, 'hard': 0, 'starved': 0}
        for fn in events:
            with open(os.path.join(evdir, fn)) as f:
                d = json.load(f)
            cv = d.get('chosen_vx', float('nan'))
            fv = d.get('fastest_feasible_vx')
            nf = d.get('n_feasible', 0)
            nt = d.get('n_twists', 0)
            if nf == 0:
                verdict, tag = '候选空间被吃光（硬约束）', 'starved'
            elif fv is not None and fv - cv > 0.3:
                verdict, tag = '★有更快可行解未选 → 软代价', 'soft'
            else:
                verdict, tag = '当时无更快可行解 → 硬约束', 'hard'
            verdicts[tag] += 1
            print('  %-22s %-8.2f %-8s %-9d %-9d %s'
                  % (fn, cv, ('%.2f' % fv) if fv is not None else '-', nf, nt, verdict))
        print('\n  汇总: 软代价(权重可调)=%d  硬约束(几何/障碍)=%d  候选枯竭=%d'
              % (verdicts['soft'], verdicts['hard'], verdicts['starved']))
        if verdicts['soft']:
            print('  → 存在软代价问题：调 critic 权重（如 PathAlign/PathDist.scale）可能有救。')
        if verdicts['hard'] >= verdicts['soft']:
            print('  → 以硬约束为主：减速由几何/障碍决定，改权重收益有限。')

    # ── 可选：逐条现场细节 ────────────────────────────────────
    if show_events and events:
        print('\n【⑤ 现场细节】')
        for fn in events[:12]:
            with open(os.path.join(evdir, fn)) as f:
                d = json.load(f)
            print('\n  ── %s ──' % fn)
            print('     t=%.1fs  位姿 odom=(%.2f,%.2f) amcl=(%.2f,%.2f)'
                  % (d.get('sim', 0), d['pose']['x'], d['pose']['y'],
                     d['pose']['amcl_x'], d['pose']['amcl_y']))
            print('     实选 vx=%.2f wz=%.2f   最快可行 vx=%s   可行/总候选=%d/%d'
                  % (d.get('chosen_vx', 0), d.get('chosen_wz', 0),
                     d.get('fastest_feasible_vx'), d.get('n_feasible', 0),
                     d.get('n_twists', 0)))
            cs = d.get('critic_scores_of_best', {})
            print('     实选轨迹的 critic: ' + '  '.join(
                '%s=%.1f' % (k, v) for k, v in cs.items()))
            cands = d.get('candidates_by_vx_desc', [])
            fast = [c for c in cands if c.get('total') is not None][:5]
            print('     最快的可行候选（vx, wz, total）: ' + '  '.join(
                '(%.2f,%.2f,%.1f)' % (c['vx'], c['wz'], c['total']) for c in fast))
    return 0


if __name__ == '__main__':
    sys.exit(main())
