#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图 ↔ 世界 刚体配准：用多位置激光样本联合求最优 origin (x, y, yaw)。

配套 tools/map_reg_scan.py 采集的 scans.npz。核心是**多位置联合拟合**——
早期的单点拟合被证明会过拟合（拿一个位置的扫描去定整张图的平移，会把该处的
局部误差当成全局平移，改完反而更糟）。此处：
  · 几十个位姿、上万个激光点一起参与，全局平移与旋转同时受限；
  · 留出法验证（拟合用 2/3，验证用 1/3），避免再过拟合。

残差的定义（关键，决定精度）
----------------------------
每个激光端点都应落在**墙面**上。把地图的「占据/自由」边界当作墙面，
残差取端点到该边界的**有符号距离**：

    sdf = 到最近占据格的距离 − 到最近自由格的距离     （占据侧为负，自由侧为正）

它恰好在边界上为 0，且**处处有梯度** —— 这正是它优于「到最近占据格距离」的地方：
后者在厚约 1.5 格的占据带内部恒为 0，形成死区，把精度锁死在 ±半格(约 1.2cm)以上；
有符号距离没有死区，配合双线性插值可做到亚格精度。

用法
----
  python3 map_reg_fit.py --scans <dir>/scans.npz --map <地图.yaml> [--out <目录>]
产出：终端报告 + <目录>/reg_report.txt + <目录>/overlay.png（拟合前后叠加图）
"""
import argparse
import os
import sys

import numpy as np
import yaml
from scipy import ndimage, optimize

from map_reg_scan import load_map


def build_sdf(occ, free, res):
    """占据/自由边界的有符号距离场（米）。occ 侧为负，free 侧为正，边界为 0。

    ⚠️ free 必须是**真正的自由格**，不能用 ~occ：本图 88% 是 unknown(205)，
    用 ~occ 会让「边界」同时出现在墙的两侧（走廊侧与墙外未知侧），
    落在墙外未知区的端点会被误判成贴合墙面。未知处置 nan，由调用方剔除。
    """
    dt_occ = ndimage.distance_transform_edt(~occ) * res      # 0 在占据格上
    dt_free = ndimage.distance_transform_edt(~free) * res    # 0 在自由格上
    sdf = dt_occ - dt_free
    sdf[~(occ | free)] = np.nan                              # 未知区：无信息
    return sdf


def endpoints(poses, scans, angles):
    """把每帧激光端点投到 odom 系。返回 (N*K, 2) 与所属帧号。

    angles 允许是 (K,) 单条（各帧共用，采集器就是这么存的）或 (N,K)。
    """
    scans = np.asarray(scans, dtype=np.float64)
    ang = np.asarray(angles, dtype=np.float64)
    if ang.ndim == 1:
        ang = np.broadcast_to(ang, scans.shape)
    pts, frame = [], []
    for k, ((lx, ly, lyaw), r, a) in enumerate(zip(poses, scans, ang)):
        good = np.isfinite(r)
        rr, aa = r[good], a[good]
        pts.append(np.stack([lx + rr * np.cos(aa + lyaw),
                             ly + rr * np.sin(aa + lyaw)], axis=1))
        frame.append(np.full(rr.size, k))
    return np.concatenate(pts), np.concatenate(frame)


def to_pixel(pts, origin, res, H):
    """odom 系点 -> 地图浮点像素坐标 (row=j, col=i)，行 0 为图顶（y 最大）。"""
    ox, oy, yaw = origin
    dx, dy = pts[:, 0] - ox, pts[:, 1] - oy
    c, s = np.cos(yaw), np.sin(yaw)
    u = dx * c + dy * s
    v = -dx * s + dy * c
    return np.stack([(H - 0.5) - v / res, u / res - 0.5], axis=1)


def residual(pts, origin, sdf, res, H, huber=0.08):
    """点在给定 origin 下的有符号距离；越界/未知处返回 nan（随后被剔除）。"""
    rc = to_pixel(pts, origin, res, H)
    val = ndimage.map_coordinates(sdf, rc.T, order=1, mode='constant', cval=np.nan)
    if huber:
        av = np.abs(val)
        # np.where 会同时求两支，小 |v| 时右支的平方根参数为负；夹到 0 以免 nan 警告
        np.maximum(2 * huber * av - huber ** 2, 0.0, out=av)
        val = np.where(np.abs(val) < huber, val, np.sign(val) * np.sqrt(av))
    return val


def cost(origin, pts, sdf, res, H):
    v = residual(pts, origin, sdf, res, H)
    v = v[np.isfinite(v)]
    if v.size < 100:
        return 1e9
    return float(np.mean(v ** 2))


def fit(pts, sdf, res, H, x0):
    """粗网格预搜（避开局部极小）→ Powell 精修 → 再重启一次。"""
    ox, oy, yaw = x0
    sub = pts[::8]
    best, bx = None, x0
    for dyaw in np.arange(-0.03, 0.0301, 0.01):
        for dx in np.arange(-0.35, 0.3501, 0.05):
            for dy in np.arange(-0.35, 0.3501, 0.05):
                cand = (ox + dx, oy + dy, yaw + dyaw)
                c = cost(cand, sub, sdf, res, H)
                if best is None or c < best:
                    best, bx = c, cand
    for _ in range(2):
        r = optimize.minimize(cost, bx, args=(pts, sdf, res, H), method='Powell',
                              options={'xtol': 1e-5, 'ftol': 1e-10, 'maxiter': 4000})
        if r.fun < best:
            best, bx = r.fun, tuple(r.x)
    return bx, best


def stats(v):
    v = v[np.isfinite(v)]
    if not v.size:
        return None
    a = np.abs(v)
    return dict(n=int(v.size), median=float(np.median(a)), mean=float(a.mean()),
                p90=float(np.percentile(a, 90)), bias=float(np.median(v)))


def report_line(tag, s):
    if s is None:
        return '  %-22s 无有效点' % tag
    return ('  %-22s n=%-6d 中位|d|=%.4fm 均值=%.4fm p90=%.4fm 偏移中位=%+.4fm'
            % (tag, s['n'], s['median'], s['mean'], s['p90'], s['bias']))


def draw_overlay(occ, res, origin0, origin1, pts, out, stride=40):
    """并排两幅：左=按原 origin 摆放，右=按拟合 origin 摆放（红=激光端点，黑=占据格）。"""
    from PIL import Image
    H, W = occ.shape
    base = np.full((H, W), 255, np.uint8)
    base[occ] = 0
    panels = []
    for org in (origin0, origin1):
        im = Image.fromarray(base).convert('RGB')
        px = im.load()
        for j, i in to_pixel(pts[::stride], org, res, H):
            ji, ii = int(i), int(j)
            if 1 <= ji < H - 1 and 1 <= ii < W - 1:
                for dj in (0, 1):
                    for di in (0, 1):
                        px[ii + di, ji + dj] = (255, 0, 0)
        panels.append(im)
    canvas = Image.new('RGB', (W * 2 + 8, H), (128, 128, 128))
    canvas.paste(panels[0], (0, 0))
    canvas.paste(panels[1], (W + 8, 0))
    canvas.save(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scans', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    out = a.out or os.path.dirname(os.path.abspath(a.scans))
    os.makedirs(out, exist_ok=True)

    d = np.load(a.scans)
    poses, scans, angles = d['poses'], d['scans'], d['angles']
    occ, res, origin, px = load_map(a.map)
    H = occ.shape[0]
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])
    free = (px >= 250)
    x0 = (ox, oy, oyaw)

    print('载入 %d 帧，共 %d 个激光点' % (len(poses), int(np.isfinite(scans).sum())))
    print('地图自由格 %d（%.1f%%），占据格 %d（%.1f%%）'
          % (int(free.sum()), 100 * free.mean(), int(occ.sum()), 100 * occ.mean()))
    sdf = build_sdf(occ, free, res)
    pts, fr = endpoints(poses, scans, angles)

    # 留出法：按帧号每隔 3 帧留 1 帧做验证
    ho = (fr % 3) == 0
    fit_mask = ~ho
    v_all0 = residual(pts, x0, sdf, res, H)
    s0 = stats(v_all0)
    # 先按初始摆放剔除离墙过远的点（多半是打进未建图区域的光线，不是墙面回波）
    near = np.isfinite(v_all0) & (np.abs(v_all0) < 0.5)

    xf, cf = fit(pts[fit_mask & near], sdf, res, H, x0)
    dx, dy, dyaw = xf[0] - ox, xf[1] - oy, xf[2] - oyaw
    print('\n初始 origin = (%.4f, %.4f, %.4f)' % x0)
    print('拟合 origin = (%.4f, %.4f, %.4f)' % xf)
    print('修正量      = Δx=%+.4f m  Δy=%+.4f m  Δyaw=%+.4f rad (%+.2f°)'
          % (dx, dy, dyaw, np.degrees(dyaw)))
    print('平移模长    = %.4f m' % np.hypot(dx, dy))

    v1 = residual(pts, xf, sdf, res, H)
    print('\n残差（激光端点到地图墙面边界的有符号距离）')
    print(report_line('全部点 · 拟合前', stats(v_all0)))
    print(report_line('全部点 · 拟合后', stats(v1)))
    print('  拟合用点（2/3）')
    print(report_line('    拟合前', stats(v_all0[fit_mask & near])))
    print(report_line('    拟合后', stats(v1[fit_mask & near])))
    print('  ★留出验证点（1/3，未参与拟合）')
    hv0, hv1 = stats(v_all0[ho & near]), stats(v1[ho & near])
    print(report_line('    拟合前', hv0))
    print(report_line('    拟合后', hv1))
    verdict = '留出集也有改善 → 不是过拟合' if (hv0 and hv1 and hv1['median'] < hv0['median']) \
        else '★留出集未改善 → 疑似过拟合，勿采用'
    print('  %s' % verdict)

    # 逐帧中位残差，看改善是否普遍（而非被少数帧拉动）
    per0, per1 = [], []
    for k in range(len(poses)):
        m = fr == k
        a0 = np.abs(v_all0[m]); a1 = np.abs(v1[m])
        a0, a1 = a0[np.isfinite(a0)], a1[np.isfinite(a1)]
        if a0.size and a1.size:
            per0.append(np.median(a0)); per1.append(np.median(a1))
    per0, per1 = np.array(per0), np.array(per1)
    if per0.size:
        print('\n逐帧中位残差：改善 %d/%d 帧，整体 %.4f → %.4f m'
              % (int((per1 < per0).sum()), per0.size, per0.mean(), per1.mean()))

    lines = ['修正量 Δx=%+.4f m  Δy=%+.4f m  Δyaw=%+.4f rad' % (dx, dy, dyaw),
             '初始 origin = (%.4f, %.4f, %.4f)' % x0,
             '拟合 origin = (%.4f, %.4f, %.4f)' % xf,
             '留出验证：%s' % verdict]
    with open(os.path.join(out, 'reg_report.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    try:
        draw_overlay(occ, res, x0, xf, pts, os.path.join(out, 'overlay.png'))
        print('\n叠加图（左=拟合前，右=拟合后；红=激光端点）：%s/overlay.png' % out)
    except Exception as e:
        print('叠加图绘制失败：%s' % e)
    return 0


if __name__ == '__main__':
    sys.exit(main())
