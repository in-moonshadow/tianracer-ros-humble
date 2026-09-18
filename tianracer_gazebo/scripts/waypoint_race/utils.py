#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_gazebo/scripts/waypoint_race/utils.py) -> ROS 2 Humble 移植。
# 读取赛道 waypoint yaml，并构造 Nav2 导航目标与 RViz 可视化 marker。
# 原版基于古月居教程（https://www.guyuehome.com/35146），move_base_msgs -> nav2_msgs。

import yaml

import geometry_msgs.msg as geometry_msgs
import visualization_msgs.msg as viz_msgs
import std_msgs.msg as std_msgs
from nav2_msgs.action import NavigateToPose

id_count = 1


def get_waypoints(filename):
    """读取 yaml 文件的 waypoints 列表。"""
    with open(filename, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data['waypoints']


def create_geometry_pose(input_pose):
    """由 waypoint 字典生成 geometry_msgs.Pose。"""
    pose = geometry_msgs.Pose()
    pose.position.x = input_pose['pose']['position']['x']
    pose.position.y = input_pose['pose']['position']['y']
    pose.position.z = input_pose['pose']['position']['z']
    pose.orientation.x = input_pose['pose']['orientation']['x']
    pose.orientation.y = input_pose['pose']['orientation']['y']
    pose.orientation.z = input_pose['pose']['orientation']['z']
    pose.orientation.w = input_pose['pose']['orientation']['w']
    return pose


def create_nav_goal(input_pose, stamp=None):
    """
    由 waypoint 字典生成 Nav2 NavigateToPose.Goal。
    :param input_pose: waypoint 字典
    :param stamp: 可选时间戳（rosgraph_msgs Time），默认不设
    """
    target = geometry_msgs.PoseStamped()
    target.header.frame_id = input_pose['frame_id']
    if stamp is not None:
        target.header.stamp = stamp
    target.pose = create_geometry_pose(input_pose)

    goal = NavigateToPose.Goal()
    goal.pose = target
    return goal


def create_viz_markers(waypoints):
    """由 waypoints 生成 RViz MarkerArray。"""
    marray = viz_msgs.MarkerArray()
    for w in waypoints:
        marray.markers.append(create_arrow(w))
        marray.markers.append(create_text(w))
    return marray


def create_marker(w):
    """生成基础 marker。"""
    global id_count
    m = viz_msgs.Marker()
    m.header.frame_id = w['frame_id']
    m.ns = w['name']
    m.id = id_count
    m.action = viz_msgs.Marker.ADD
    m.pose = create_geometry_pose(w)
    m.scale = geometry_msgs.Vector3(x=1.0, y=0.3, z=0.3)
    m.color = std_msgs.ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

    id_count = id_count + 1
    return m


def create_arrow(w):
    """生成箭头 marker。"""
    m = create_marker(w)
    m.type = viz_msgs.Marker.ARROW
    m.color = std_msgs.ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    return m


def create_text(w):
    """生成文字 marker。"""
    m = create_marker(w)
    m.type = viz_msgs.Marker.TEXT_VIEW_FACING
    m.pose.position.z = 2.5
    m.text = w['name']
    return m
