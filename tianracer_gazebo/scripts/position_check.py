#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_gazebo/utils/position_check.py) -> ROS 2 Humble 移植。
# 检查点（赛道门）通过判定：车本帧运动轨迹线段与检查门线段是否相交。
# 核心算法 is_intersect / cal_distance / load_checkpoint 与 ROS1 原版完全一致，
# 仅将 analysis() 的输入从 gazebo ModelStates 消息改为 (x, y) 位置，由调用方（judge_system）提供。

import os
import sys

from ament_index_python.packages import get_package_share_directory

# 使同目录脚本与 waypoint_race 子包可被 import（ament_cmake 安装后脚本位于 lib/<pkg>/ 下）
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import waypoint_race.utils as utils

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")
robot_name = os.getenv("TIANBOT_NAME", os.getenv("TIANRACER_NAME", ""))

ana_cnt = 0
ana_check_list = []
ana_history_position = (0, 0)


def load_checkpoint(file_name):
    """
    从 yaml 文件加载检查点，两两配对成线段存入全局 ana_check_list。
    :param file_name: check_points yaml 文件路径
    """
    global ana_check_list

    _checkpoint = utils.get_waypoints(file_name)

    for _count in range(len(_checkpoint)):
        _point = _checkpoint[_count]
        _point_x = utils.create_geometry_pose(_point).position.x
        _point_y = utils.create_geometry_pose(_point).position.y

        if _count % 2 == 0:
            # 偶数下标：开始新线段
            _pair = (_point_x, _point_y),
        else:
            # 奇数下标：补全线段并加入列表
            _pair = _pair + ((_point_x, _point_y),)
            ana_check_list.append(_pair)


def is_intersect(line1, line2):
    """
    判断两条线段是否相交。
    :param line1, line2: 线段两端点二元组，如 ((-0.85,-0.57), (-1.91,-0.54))
    :return: True / False
    """
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    # 判断两线段是否平行
    if (y4 - y3) * (x2 - x1) == (y2 - y1) * (x4 - x3):
        return False

    # 判断点是否在线段上
    def is_on_segment(x, y, x1, y1, x2, y2):
        return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

    # 判断线段是否相交
    denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denominator
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator
    if 0 <= ua <= 1 and 0 <= ub <= 1:
        return True
    elif is_on_segment(x1, y1, x3, y3, x4, y4) or is_on_segment(x2, y2, x3, y3, x4, y4):
        return True
    else:
        return False


def cal_distance(points):
    """计算两点 L2 距离。"""
    point1, point2 = points
    x1, y1 = point1
    x2, y2 = point2
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def analysis(position):
    """
    检测车本帧是否压过检查门线。
    :param position: (x, y) 车当前位置
    :return: (检查点信息, 本帧移动距离) 或 (None, 本帧移动距离)
    """
    global ana_history_position, ana_cnt, ana_check_list

    x, y = position
    now_line = (ana_history_position, (x, y))
    distance = cal_distance(now_line)
    ana_history_position = (x, y)

    if not ana_check_list:
        return None, distance

    tmp = ana_cnt % 3
    if is_intersect(now_line, ana_check_list[tmp]):
        print('已经通过' + str(tmp) + '点')
        ana_cnt += 1
        return ('已经通过' + str(tmp) + '点', ana_cnt - 1), distance
    return None, distance


def reset_variables():
    """重置检查点计数与历史位置。"""
    global ana_history_position, ana_cnt
    ana_cnt = 0
    ana_history_position = (0, 0)


def load_checkpoint_from_world():
    """按 TIANRACER_WORLD 环境变量定位并加载 check_points.yaml。"""
    pkg_path = get_package_share_directory("tianracer_gazebo")
    filename = os.path.join(pkg_path, "waypoint_race", f"{world}_check_points.yaml")
    print(f"\033[1;33mThe {world}.world is loading...\033[0m")
    load_checkpoint(filename)
