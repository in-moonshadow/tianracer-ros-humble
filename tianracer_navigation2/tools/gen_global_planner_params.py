#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 DWB 定稿基线（navfn_dwb_nav2_params.yaml）生成「只换全局规划器」的对照参数。

设计原则 —— 单变量：
  · 以定稿 DWB 为基底（局部规划器、costmap、AMCL、smoother 全部原样保留），
    只替换 planner_server 的 GridBased 段。
  · 这样 DWB 定稿的 min_vel_x 0.6、robot_radius 0.07、inflation 0.3/10.0
    等全部已调优项仍然生效，成绩差异可归因到全局规划器本身。
  · 现有仓库里的 theta_star_*/smac_* 变体**不能直接用**：它们是未调优模板
    （robot_radius 0.3、inflation_radius 0.5、cost_scaling_factor 3.0，
    smac 那个还用 allow_unknown: true），照搬会重新引入已解决的
    「贴墙进死区 → No valid trajectories」问题（见 DWB 文档第 7.1 节）。

运行：python3 tools/gen_global_planner_params.py
产出：
  params/theta_star_dwb_nav2_params.yaml   —— Theta* 全局 + DWB 局部
  params/smac_dwb_nav2_params.yaml         —— Smac Lattice 全局 + DWB 局部
"""

import os
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(TOOLS)
PARAMS = os.path.join(PKG, 'params')
BASE = os.path.join(PARAMS, 'navfn_dwb_nav2_params.yaml')

HEADER = """# 全局规划器对照参数：{desc}
# ---------------------------------------------------------------------------
# 本文件由 tools/gen_global_planner_params.py 自动生成，**勿手改**。
# 基底 = params/navfn_dwb_nav2_params.yaml（DWB 定稿），只替换 GridBased 段。
# 目的：在局部规划器、costmap、AMCL、smoother 完全不变的前提下，
#       单独评估「NavFn 栅格折线路径」是否为竞速瓶颈。
#
# 运行：ros2 launch tianracer_navigation2 nav2.launch.py use_planner={planner}
# ---------------------------------------------------------------------------

"""

# 各全局规划器的 GridBased 段。缩进与基线一致（4 空格起，子键 6 空格）。
GRIDBASED = {}

GRIDBASED['theta_star'] = """    GridBased:
      plugin: "nav2_theta_star_planner/ThetaStarPlanner"
      # Theta* 的核心优势：路径是**任意角度的直线段**（任意两个可达栅格间直接连线），
      # 而非 NavFn 的 8 邻域栅格折线。理论上在梳齿蛇形区能少走锯齿路。
      #   how_many_corners 8 = 允许 8 邻域扩展（4 会退化成只能走正交方向）
      #   w_euc_cost 1.0 / w_traversal_cost 2.0 用官方默认
      #   use_final_approach_orientation false = 不强求终点朝向（与 DWB 的
      #     xy_goal_tolerance 0.25 配合，避免为对准朝向产生额外绕行）
      #   allow_unknown false = 与 DWB 定稿一致（本图 87.9% 未知区，必须关）
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      use_final_approach_orientation: false
      allow_unknown: false
      tolerance: 0.1
"""

GRIDBASED['smac'] = """    GridBased:
      # Smac Lattice：按给定状态格图（state lattice）搜索，**原生建模阿克曼运动基元**。
      # 这是与 NavFn/Theta* 的本质区别——后两者的路径不保证能被阿克曼车跟踪。
      # 格图文件 smac_lattice_ackermann.json 来自仓库既有变体（smac_graceful）。
      #
      # ⚠️ allow_unknown 必须为 false：仓库既有 smac 变体写的是 true，
      #    而本图 87.9% 为未知区，true 会让规划器穿未知区抄近路、绕开赛道与检查门。
      plugin: "nav2_smac_planner/SmacPlannerLattice"
      allow_unknown: false
      tolerance: 0.25
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      analytic_expansion_max_cost: 200.0
      analytic_expansion_max_cost_override: false
      reverse_penalty: 2.0
      change_penalty: 0.05
      non_straight_penalty: 1.05
      cost_penalty: 2.0
      rotation_penalty: 5.0
      retrospective_penalty: 0.015
      lattice_filepath: $(find-pkg-share tianracer_navigation2)/params/smac_lattice_ackermann.json
      lookup_table_size: 20.0
      cache_obstacle_heuristic: false
      allow_reverse_expansion: false
      coarse_search_resolution: 1
      goal_heading_mode: "DEFAULT"
      smooth_path: True
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2
"""

VARIANTS = [
    ('theta_star', 'theta_star_dwb', 'Theta* 全局规划器 + DWB 局部规划器（定稿）'),
    ('smac', 'smac_dwb', 'Smac Lattice 全局规划器 + DWB 局部规划器（定稿）'),
]


def main():
    with open(BASE, encoding='utf-8') as f:
        lines = f.readlines()

    # 定位 GridBased 段：从 '    GridBased:' 到 'planner_server_rclcpp_node:'
    start = end = None
    for i, ln in enumerate(lines):
        if ln.rstrip('\n') == '    GridBased:' and start is None:
            start = i
        if start is not None and ln.startswith('planner_server_rclcpp_node:'):
            end = i
            break
    if start is None or end is None:
        print('!! 未能定位 GridBased 段（start=%s end=%s）' % (start, end), file=sys.stderr)
        return 1
    print('基线 GridBased 段：第 %d~%d 行（共 %d 行）' % (start + 1, end, end - start))

    for key, out, desc in VARIANTS:
        header = HEADER.format(desc=desc, planner=out)
        grid = GRIDBASED[key]
        out_lines = [header] + lines[:start] + [grid, '\n'] + lines[end:]
        path = os.path.join(PARAMS, out + '_nav2_params.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            f.writelines(out_lines)
        print('  ✔ %s_nav2_params.yaml' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
