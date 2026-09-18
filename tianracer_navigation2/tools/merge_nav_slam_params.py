#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
合并 Nav2 参数与 slam_toolbox 参数为单份文件，并把 `<robot_namespace>` 占位符替换掉。

为什么需要它（两个都是实测踩出来的）：
  1. 我们的 `navfn_dwb_nav2_params.yaml` 里有 `<robot_namespace>/xxx` 占位符，
     由 tianracer 自己的 launch 在带 namespace 时替换。直接喂给 nav2_bringup 会报
     `Invalid topic name: topic name must not contain characters other than alphanumerics...`
     → local_costmap configure 抛异常 → 导航栈起不来。
  2. `nav2_bringup/slam_launch.py` 用 `HasNodeParams(params_file, node_name='slam_toolbox')`
     判断是否带参数启动 slam_toolbox。若参数文件里没有 `slam_toolbox:` 段，它走「不带参数」
     分支并**忽略** `slam_params_file` → lifecycle_manager_slam 只配置 map_saver，
     **从不 configure/activate slam_toolbox** → 进程退出、map 从未发布。
  故必须把 slam_toolbox 段合并进同一份文件（两个 launch 共用）。

用法：
  python3 merge_nav_slam_params.py <nav_params> <slam_params> <out_yaml> [namespace]

合并方式：顶层「节点名 -> ros__parameters」字典合并（不是文本拼接，避免缩进/文档分隔冲突）。
同名顶层键以 nav 侧为准（本仓库中两者无重名节点）。
"""

import os
import sys

import yaml


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    nav_f, slam_f, out_f = sys.argv[1], sys.argv[2], sys.argv[3]
    ns = sys.argv[4] if len(sys.argv) > 4 else os.environ.get('TIANBOT_NAME', '')
    if ns in ('', '/'):
        ns = ''

    def load(p):
        with open(p, encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    nav, slam = load(nav_f), load(slam_f)
    merged = {}
    for k, v in slam.items():
        merged[k] = v
    for k, v in nav.items():
        merged[k] = v          # nav 侧优先（无重名，保险起见）

    # 替换占位符（递归处理字符串）
    def sub(o):
        if isinstance(o, str):
            if ns:
                return o.replace('<robot_namespace>', ns)
            # 无 namespace：把 `/x` 与 `x/` 形式的前缀整体去掉
            return (o.replace('/<robot_namespace>/', '/')
                     .replace('<robot_namespace>/', '')
                     .replace('/<robot_namespace>', ''))
        if isinstance(o, dict):
            return {k: sub(v) for k, v in o.items()}
        if isinstance(o, list):
            return [sub(x) for x in o]
        return o

    merged = sub(merged)

    # 自检：必须同时含有两侧的关键节点段，且无残留占位符
    text = yaml.safe_dump(merged, allow_unicode=True)
    if '<robot_namespace>' in text:
        print('!! 仍有未替换的占位符', file=sys.stderr)
        return 1
    for need in ('slam_toolbox', 'controller_server', 'planner_server', 'bt_navigator'):
        if need not in merged:
            print('!! 合并结果缺少 %s 段' % need, file=sys.stderr)
            return 1

    with open(out_f, 'w', encoding='utf-8') as f:
        f.write(text)
    print('已生成: %s（顶层段: %s）' % (out_f, ', '.join(sorted(merged))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
