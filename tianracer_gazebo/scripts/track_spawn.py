#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""各赛道的出生位姿单一来源（single source of truth）。

出生位姿有三个消费方，必须一致，否则 AMCL 会从错误位姿起算、且裁判 reset 会把车
送回错误位置（AMCL 的随机粒子注入在本仓库被关闭：recovery_alpha_slow/fast = 0.0，
故初始位姿错了无法自愈，实测 16m 偏差必然发散）：
  ① gz 里车的实际生成点   —— tianracer_on_racetrack.launch.py 的 world:= 参数
  ② AMCL 的初始位姿       —— nav2 params 的 amcl.initial_pose.*（由 nav2.launch.py 注入）
  ③ 裁判 reset 送回的点    —— judge_system.py 的 init_x/init_y/init_z/init_yaw

历史上这三处各写一份硬编码值，换赛道时极易漏改（2026-09-14 就发生过：只改了 ①，
导致 racetrack_1 / raicom 的车生成在 16m / 6.7m 外而 AMCL 仍以为在原点）。
本模块把位姿收敛到唯一一张表，三处都从这里取。

用法：
    from track_spawn import get_spawn
    x, y, z, yaw = get_spawn('test_indoor')       # 裸名也接受
    x, y, z, yaw = get_spawn('test_indoor.world') # 带后缀也接受

⚠️ z 必须 ≥ 该赛道地板顶面，否则车生成在地板实体内部、被物理引擎弹到地板【下方】，
   表现为 Gazebo 里看不到车、雷达被埋扫不到墙。地板顶面是【实测值】（丢车法），
   不要按 DAE 包围盒静态推断——DAE 的 <node> 矩阵读法易错，实测才有准。
"""

import math

# 未登记的赛道回退到这里。与 tianracer_racetrack.world 一致，也是 launch 的 world 默认值。
DEFAULT_TRACK = 'tianracer_racetrack'
DEFAULT_SPAWN = (0.0, 0.0, 0.1, 1.54)

# key 用【裸赛道名】（不带 .world），与 judge_system.py 的 world 参数、
# dwb_round.sh 的 $WORLD 同形；get_spawn 也接受带 .world 的形态。
# 位姿 = (x, y, z, yaw)，单位 m / rad，坐标系为 gz 世界系（= map 系）。
TRACK_SPAWNS = {
    # 无 mesh 地板（ground_plane 在 z=0）。内嵌 start_plane 发车线
    # （world 内 pose 0.084155 0.911194），车头朝赛道行进方向 -y。
    'race_with_cones': (0.084155, 0.911194, 0.1, -math.pi / 2),

    # 地板顶面实测 0.5（实心厚板）。z=0.55 = 顶面 +0.05 余量，轻落后落在板上。
    # 生成点 (0,0) 实测落在一条 1.03m 宽、南北向走廊正中（西墙 x=-0.638~-0.488、
    # 东墙 x=+0.544~0.694）。★ 该点沿 ±x 仅 0.5m 即撞墙，故出生朝向必须沿 ±y：
    # yaw = +π/2 朝北（门序 y 递减 门1(7.9)→门2(2.73)→门3(-0.14)，先北上去门1）。
    # 注意：这里【不能】借用路点0 的朝向——路点0 在 (-2.146, 4.458)，与出生点 (0,0)
    # 是两个不同位置，其朝向在 (0,0) 处恰好正对东墙（曾因此导致车头朝墙）。
    'test_indoor': (0.0, 0.0, 0.55, math.pi / 2),

    # 地板顶面实测 0.5。生成点【就是】本赛道路点0 (15.974746,1.153895)（实测在地板上），
    # 故朝向可直接取路点0 朝向 -1.6300(-93.4°)；实测该朝向正前方 8m 无墙 ✅。
    # 判据：只有「出生点 = 路点0」时才能借用路点0 朝向（见 test_indoor 的反例）。
    'racetrack_1': (15.974746, 1.153895, 0.55, -1.6300),

    # 地板顶面实测 0.818（其 DAE 名义厚 2.0，实际碰撞面 0.818）。
    # 生成点 = 起跑线（棋盘格）：(0,0) 就在这条线上（距棋盘格中心 0.11m），
    # 迁移时误取「路点0」当出生点（4.099202,5.312033），偏出 6.73m ⇒ 车不在起跑线。
    # 已用 贴图像素->世界 映射三重验证：9/9 路点落白赛道、出生点落白区、棋盘格处 z=0.818
    # 与独立实测一致。起跑线中心实测 (0.112,-0.001)，与 (0,0) 同属一格自由空间
    # （地图 px(1000,983) gray=254），故取 (0,0) 而不用 0.112，少一处魔法数。
    # yaw=1.54(88.2°)，也是该处南北向车道走向（射线投射 +90° 自由 6.05m，
    # ±0° 仅 0.60m；门2 线段方向 -1.3° 东西横跨，法向即 ±90°）。
    # ⚠️ 该赛道另有上游缺陷（锥体 model:// 路径 + <scale>10，放大后 19.8m 见方覆盖门1），
    #    暂不宜竞速，故本出生点【未经实跑验证】。
    'raicom': (0.0, 0.0, 0.868, 1.54),

    # 无 mesh 地板；显式列出以便检索（与 DEFAULT_SPAWN 相同）。
    'tianracer_racetrack': DEFAULT_SPAWN,
}


def get_spawn(world):
    """取 world 的出生位姿 (x, y, z, yaw)；未登记则回退 DEFAULT_SPAWN。

    world 接受裸名（'test_indoor'）或带后缀（'test_indoor.world'）两种形态，
    以便调用方直接用自己手上的字符串，不必各自转换（转换点多了就会漏）。
    """
    if not world:
        return DEFAULT_SPAWN
    key = world[:-len('.world')] if world.endswith('.world') else world
    return TRACK_SPAWNS.get(key, DEFAULT_SPAWN)
