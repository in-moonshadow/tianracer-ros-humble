#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""以 navfn_dwb_nav2_params.yaml 为基线，生成 MPPI 的两份配置。

做法：逐行复制基线文件，只把 `FollowPath:` 段（从 `    FollowPath:` 到
`controller_server_rclcpp_node:` 之前）整体替换成 MPPI 的 FollowPath 段。
这样其余所有已调好的公共段（velocity_smoother / costmap / AMCL 等）逐字节保留，
使 DWB 与 MPPI 的对比是「只差局部规划器」的干净对比。

生成：
  navfn_mppi_nav2_params.yaml      —— MPPI，允许倒车（vx_min < 0）
  navfn_mppi_fwd_nav2_params.yaml  —— MPPI，禁止倒车（vx_min = 0）

用法：python3 tools/gen_mppi_params.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, '..', 'params')
BASE = os.path.join(PARAMS, 'navfn_dwb_nav2_params.yaml')

HEADER = """# =============================================================================
# MPPI 局部规划器参数（{variant_desc}）
# =============================================================================
# 本文件由 tools/gen_mppi_params.py 从 navfn_dwb_nav2_params.yaml 生成：
#   **除 controller_server.FollowPath 段外，其余与 DWB 文件逐字节相同**
#   —— velocity_smoother / costmap（robot_radius 0.07、local 6x6）、AMCL、
#      planner_server（NavFn）、各 use_sim_time 等公共段原样保留。
# 目的：让「DWB vs MPPI」的对比只差局部规划器本身，不掺其他配置差异。
#   重新生成：python3 tianracer_navigation2/tools/gen_mppi_params.py
#
# 运行：ros2 launch tianracer_navigation2 nav2.launch.py use_planner={planner_name}
#
# 【与 DWB 文件的关键对齐点（速度直接到位）】
#   vx_max 1.5 / wz_max 3.95  —— 对齐 DWB 定稿（= 1.5·tan(0.6)/0.26）
#   motion_model Ackermann + min_turning_r 0.38
#                            —— 0.38 是本项目的物理极限（见下面 FollowPath 段注释）
#   {vx_min_desc}
# =============================================================================
"""

# MPPI 的 FollowPath 段。缩进与基线一致（4 空格起，子键 6 空格）。
FOLLOWPATH_TMPL = """    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # ---- 采样与优化核心 ----
      # time_steps x model_dt = 预测时域 = 56 x 0.05 = 2.8s
      #   比 DWB 的 sim_time 0.8s 长 3.5 倍 —— MPPI 的核心优势（看得更远）。
      #   DWB 的地平线受「sim_time x vx」耦合限制（快轨迹长、慢轨迹短，导致
      #   弯中停车更划算），MPPI 的时域不随 vx 变化，理论上不会重现该机制。
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000          # 每周期采样 2000 条候选（DWB 是 420 条）
      iteration_count: 1        # MPPI 官方建议保持 1；>1 收益小且耗时
      # ---- 采样噪声标准差（探索幅度）----
      # vx_std 曾试 0.1（2026-09-14）以消除「爬行段」，**部分成功但总体退步，已回退**：
      #   爬行段(0.05~0.3) 确实从 7.5% 降到 1.9%（目标达成），
      #   但 >=1.0 占比从 32.4% 掉到 **0%**，里程 25.08 -> 20.54m。
      #   机制：vx_std 是采样探索幅度，收窄后 MPPI 难以探索到高速候选，
      #   softmax 平均随之被拉低 —— 它与 deadband 是**对抗关系**
      #   （deadband 把速度往高推，vx_std 决定能否采到高速样本）。
      #   故保持官方默认 0.2，速度靠 deadband 抬。
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      # ---- 速度/加速度硬边界 ----
      # 对齐 DWB 定稿：vx_max 1.5 / wz_max 3.95 = 1.5·tan(0.6)/0.26，
      #   即 transform.py 舵角钳位 (±0.6rad) 与轴距 0.26 决定的上限。
      # ── 实验⑦（2026-09-18）：高速档 vx_max 3.0 / wz_max 7.89 / deadband 2.5 ──
      #   **实测失败已回退**（mppi2，默认赛道可视化跑，0/9）：车撞墙、速度失控
      #   （速度中位 0.300、最大 2.110 m/s 剧烈振荡），里程仅 10.24m，
      #   `Optimizer fail to compute path` ×9、`follow_path Aborting handle` ×9。
      #   机制：deadband 是**全局统一的速度设定点**，权重 35 对每一步、每个位置都施加；
      #   弯道里 CostCritic（3.81）压不住它 ⇒ 车被推进弯道贴墙 ⇒ 优化器无可行解。
      #   与 DWB 的 min_vel_x 同属「一个全局速度设定点无法同时服务直道与弯道」，
      #   只是 MPPI 表现为失控而非"无可行解帧"。历史对照：deadband 1.0 时巡航
      #   0.813 m/s、最好 3/9、从未走通 9 门；而 DWB 是 2.495 m/s 稳定 9/9
      #   ⇒ **MPPI 提速方向关闭，别再试中间值**。
      # ⚠️ 该轮的另一处教训：本生成器基座是**通用** navfn_dwb 文件（smoother 线速度
      #   1.5），实验⑦ 曾把 vx_max 抬到 3.0 却漏改 smoother，使「规划器以为能跑
      #   3.0、实际被钳到 1.5」——mppi1 轮因此作废（cmd_vx 上限恰好 1.500、
      #   27.6% 的帧贴着它）。今后再动 vx_max，务必同步改公共段的
      #   velocity_smoother max|min_velocity[0]。
      vx_max: 1.5
      vx_min: {vx_min}
      vy_max: 0.5
      wz_max: 3.95
      ax_max: 3.0
      ax_min: -3.0
      ay_min: -3.0
      ay_max: 3.0
      az_max: 3.5
      # ---- 软最大熵加权（MPPI 特有）----
      # temperature 越小越"贪心"选最优候选（趋于确定性）；越大越平滑但迟钝。
      # gamma 是候选代价的折扣/归一化系数。
      # 曾试 0.15（2026-09-14）以缩短起步爬升，**两轮无收益已回退**：
      #   机制假设成立——起步首次运动确实由 4.32s 提前到 2.97s（达 DWB 的 2.95s），
      #   因为 MPPI 的慢起步不是加速度限制（ax_max 3.0 已宽于 DWB 的 acc_lim_x 2.5），
      #   而是 **softmax 平均**：零速初值时 2000 条候选多数接近 0，被平均掉。
      #   但首门成绩反过来变差（t1 2/9，t2 0/9，且 t2 里程仅 12.77m 卡在北侧弯），
      #   而 temp 0.3 的四轮为 2/3/0/0。两轮不足以定论方差，且首门余量仍卡在 0~2.4s，
      #   故回退官方默认 0.3。
      temperature: 0.3
      gamma: 0.015
      # ---- 运动模型 ----
      # Ackermann：显式建模阿克曼约束（DWB 靠 vtheta 采样间接近似）。
      motion_model: "Ackermann"
      AckermannConstraints:
        min_turning_r: 0.38     # 本项目物理极限：L/tan(0.6) = 0.26/0.684 = 0.38
      # ---- 路径与可视化 ----
      # prune_distance 曾试 3.0（2026-09-14）以抬高参考路径末端、期望抬高巡航速度，
      # **实测证伪**：prune 1.7 与 3.0 的开阔段巡航都是 0.55~0.65，无语义差别
      # （a1@1.7 直道 0.592，f1@3.0 直道 0.55~0.67）。曾据 a1「±0.001 窄带」
      # 与 1.7/2.8=0.607 的 2.5% 吻合推断「vx* = prune/时域」是控制平衡点，
      # 该推断被 f1 推翻——那个窄带只是那段恰好直行的巧合，属过度解读。
      # 且 prune 3.0 + 允许倒车（b1）会诱发**全程倒车 3.4m**（cmd_vx 均值 -0.107，
      # 走向 -Y），而 1.7 + 允许倒车（a1）是向前。故退回 1.7。
      prune_distance: 1.7
      transform_tolerance: 0.1
      visualize: false
      reset_period: 1.0         # (only in Humble)
      regenerate_noises: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      TrajectoryValidator:
        plugin: "mppi::DefaultOptimalTrajectoryValidator"
        collision_lookahead_time: 2.0
        consider_footprint: false
      # ---- critics ----
      # 与 DWB 的五个 critic 大致对应：
      #   GoalCritic/GoalAngleCritic   <-> GoalDist
      #   PathAlignCritic/PathFollowCritic/PathAngleCritic <-> PathAlign/PathDist
      #   CostCritic                   <-> BaseObstacle
      #   PreferForwardCritic          <-> （DWB 无对应；对应 TEB 的
      #                                     weight_kinematics_forward_drive，
      #                                     即"偏好前进"的软手段）
      #   ConstraintCritic             <-> （DWB 无对应；软惩罚越界）
      #   VelocityDeadbandCritic       <-> min_vel_x（★关键：DWB 的速度地板）
{critics_list}
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: {prefer_fwd_weight}
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        # 必须为 false：本项目的 local_costmap 用 robot_radius 0.07（圆形），
        # **没有 footprint 多边形**。设 true 会让 controller_server 在 configure
        # 阶段直接抛 "Considering footprint in collision checking but no robot
        # footprint provided in the costmap" 并 bringup 失败（实测）。
        consider_footprint: false
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      PathAlignCritic:
        enabled: true
        cost_power: 1
        # 判别实验：14.0 -> 0.0（等价禁用）。假设：prune_distance 1.7m 截断参考路径，
        # 而轨迹长度=时域(2.8s)x vx，vx>0.6 起评价点落到路径末端之外，
        # PathAlign 惩罚随 vx 单调上升 -> 与 deadband(推高) 在 0.8~1.0 形成平衡
        # （实测 vx 众数 0.8~1.0、从未 >1.2，恰在死区下沿，与此吻合）。
        # 若禁用后速度上升到 1.2+，假设成立；若不变，压制来自其他 critic。
        cost_weight: 0.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
      # ★ VelocityDeadbandCritic —— DWB min_vel_x 的 MPPI 等价物（2026-09-14 加入）
      # 源码（velocity_deadband_critic.cpp，已核）：
      #   cost += sum( max(|deadband_vx| - |vx|, 0) ) * model_dt * weight
      # 即**惩罚 |vx| < deadband_vx 的候选**，把速度推离低速区。
      #
      # 为什么必须加：MPPI 架构上**没有速度地板**——
      #   DWB  : 固定速度采样对（min_vel_x 0.6 … max_vel_x 1.5）+ 硬地板
      #   MPPI : 高斯采样（vx_std 0.2）+ softmax 软平均，无任何下限机制
      # 实测对照（同一条赛道、同一卡点）：
      #   开阔段  DWB 1.29  vs  MPPI 0.55~0.65
      #   窄弯段  DWB 1.50  vs  MPPI 0.03~0.19（掉速后触发 Failed to make progress → abort）
      #   cmd_vx vs |wz| 分桶：DWB 全程 1.24~1.41 平坦（地板撑住）；
      #                       MPPI 直行 0.56 → w=0.1 桶 0.24 → w=0.2 桶 0.28
      # 注意 deadband 是 |vx| 上的死区：对 vx_min<0 的允许倒车组，低速倒车同样被罚
      # （官方文档：a zero array means the critic will take no action）。
      VelocityDeadbandCritic:
        enabled: true
        cost_power: 1
        cost_weight: 35.0            # 官方默认；比其余 critic 高一个量级，属预期
        # 历史实测映射：deadband 0.6 -> cmd_vx 均值 0.593；1.0 -> 0.813（单调）。
        # ★ 实验⑦ 曾试 2.5（2026-09-18）：**失败已回退**——车撞墙失控、0/9。
        #   它是全局统一的速度设定点，弯道里压不住 CostCritic（见上「速度硬边界」段）。
        deadband_velocities: [1.0, 0.0, 0.0]

"""

VARIANTS = [
    {
        'out': 'navfn_mppi_nav2_params.yaml',
        'planner_name': 'navfn_mppi',
        'variant_desc': '允许倒车组',
        'vx_min': '-0.35',
        'vx_min_desc': 'vx_min -0.35 —— **允许倒车**（历史较差）',
        'critics_list': '      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", '
                        '"PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic", "VelocityDeadbandCritic"]',
        'prefer_fwd_weight': '5.0',
    },
    {
        'out': 'navfn_mppi_fwd_nav2_params.yaml',
        'planner_name': 'navfn_mppi_fwd',
        'variant_desc': '禁止倒车组',
        'vx_min': '0.0',
        'vx_min_desc': 'vx_min 0.0 —— **禁止倒车**（历史上更优），只能在 [0, 1.5] 内采样',
        'critics_list': '      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", '
                        '"PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic", "VelocityDeadbandCritic"]',
        'prefer_fwd_weight': '5.0',
    },
]


def main():
    with open(BASE, encoding='utf-8') as f:
        lines = f.readlines()

    # 定位 FollowPath 段：从 '    FollowPath:' 到 'controller_server_rclcpp_node:'
    start = end = None
    for i, ln in enumerate(lines):
        if ln.rstrip('\n') == '    FollowPath:' and start is None:
            start = i
        if start is not None and ln.startswith('controller_server_rclcpp_node:'):
            end = i
            break
    if start is None or end is None:
        print('!! 未能定位 FollowPath 段（start=%s end=%s）' % (start, end), file=sys.stderr)
        return 1
    print('基线 FollowPath 段：第 %d~%d 行（共 %d 行）' % (start + 1, end, end - start))

    for v in VARIANTS:
        header = HEADER.format(**v)
        follow = FOLLOWPATH_TMPL.format(**v)
        path = os.path.join(PARAMS, v['out'])
        with open(path, 'w', encoding='utf-8') as f:
            f.write(header)
            f.writelines(lines[:start])
            f.write(follow)
            f.writelines(lines[end:])
        print('  ✔ %s' % v['out'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
