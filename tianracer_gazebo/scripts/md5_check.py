#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 1 (tianracer_gazebo/scripts/md5.so) -> ROS 2 Humble 纯 Python 重写。
# md5.so 是 Cython 编译的模型完整性校验工具（python3.8 ABI，无法在 python3.10 加载）。
# 接口重建：md5sum / format_hash / hash_file / hash_folder / verify_hash。
#
# 官方模型哈希清单从 md5.so 二进制内嵌改为可维护的 yaml：
#   <pkg>/waypoint_race/official_model_hash.yaml
# 其中 key 为相对路径（相对 tianracer_gazebo 包根），value 为 32 位小写 hex md5。

import hashlib
import os

import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = "tianracer_gazebo"

# 需要校验的模型/世界/URDF 文件（相对包根，来自 md5.so 内嵌清单）
OFFICIAL_FILES = [
    "urdf/macros_tf.xacro",
    "urdf/macros.xacro",
    "urdf/racecar.gazebo",
    "urdf/racecar.urdf.gazebo",
    "urdf/tianracer_run.urdf.xacro",
    "urdf/tianracer_run.xacro",
    "urdf/tianracer.urdf.xacro",
    "urdf/tianracer.xacro",
    "worlds/racetrack_1/meshes/base_link.dae",
    "worlds/racetrack_1/meshes/base_link.stl",
    "worlds/racetrack_1/meshes/ground.png",
    "worlds/racetrack_1/meshes/wall_link.stl",
    "worlds/racetrack_1/meshes/wall.stl",
    "worlds/racetrack_1/model.config",
    "worlds/racetrack_1/model.sdf",
    "worlds/racetrack_1.world",
    "worlds/race_with_cones.world",
    "worlds/raicom/meshes/base_link.dae",
    "worlds/raicom/meshes/ground.png",
    "worlds/raicom/model.config",
    "worlds/raicom/model.sdf",
    "worlds/raicom.world",
    "worlds/room_mini.world",
    "worlds/test_indoor/meshes/base_link.dae",
    "worlds/test_indoor/meshes/base_link.stl",
    "worlds/test_indoor/meshes/ground.png",
    "worlds/test_indoor/meshes/wall_link.stl",
    "worlds/test_indoor/model.config",
    "worlds/test_indoor/model.sdf",
    "worlds/test_indoor.world",
    "worlds/tianracer_racetrack/meshes/aa2_iflytek.png",
    "worlds/tianracer_racetrack/meshes/aa3_iflytek.png",
    "worlds/tianracer_racetrack/meshes/aa3.png",
    "worlds/tianracer_racetrack/meshes/base_link.dae",
    "worlds/tianracer_racetrack/meshes/base_link_iflytek.dae",
    "worlds/tianracer_racetrack/meshes/base_link.STL",
    "worlds/tianracer_racetrack/meshes/wall.STL",
    "worlds/tianracer_racetrack/model.config",
    "worlds/tianracer_racetrack/tianracer_racetrack.sdf",
    "worlds/tianracer_racetrack.world",
]


def md5sum(s):
    """对字符串计算 md5。"""
    return hashlib.md5(s.encode("utf-8")).hexdigest()


def format_hash(raw):
    """将 bytes/hexdigest 规范化为 32 位小写 hex 字符串。"""
    if isinstance(raw, bytes):
        return raw.hex()
    return str(raw).lower()


def hash_file(file_path):
    """对单个文件内容计算 md5，返回 32 位小写 hex。"""
    h = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return format_hash(h.hexdigest())


def hash_folder(folder, relative_files):
    """
    计算文件夹下指定相对文件列表的 md5，返回 {相对路径: hex} 字典。
    :param folder: 包根目录（绝对路径）
    :param relative_files: 相对路径列表
    """
    result = {}
    for rel in sorted(relative_files):
        full = os.path.join(folder, rel)
        if not os.path.isfile(full):
            result[rel] = None
            continue
        result[rel] = hash_file(full)
    return result


def load_official_hash():
    """加载官方模型哈希清单（yaml），缺失则返回空 dict。"""
    pkg_share = get_package_share_directory(PACKAGE)
    yaml_path = os.path.join(pkg_share, "waypoint_race", "official_model_hash.yaml")
    if not os.path.isfile(yaml_path):
        return {}
    with open(yaml_path, "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader) or {}
    return data


def verify_hash(folder, relative_files=None):
    """
    校验模型文件完整性，返回 (passed: bool, detail: dict)。
    :param folder: 包根目录
    :param relative_files: 待校验相对路径（默认 OFFICIAL_FILES）
    """
    relative_files = relative_files or OFFICIAL_FILES
    official = load_official_hash()
    current = hash_folder(folder, relative_files)

    passed = True
    detail = {}
    for rel in sorted(relative_files):
        official_h = official.get(rel)
        current_h = current.get(rel)
        if official_h is None:
            # 清单中无此文件的官方值：视为不校验（不阻断）
            detail[rel] = {"status": "skipped", "hash": current_h}
            continue
        if current_h is None:
            detail[rel] = {"status": "missing", "hash": None}
            passed = False
            continue
        ok = (format_hash(official_h) == format_hash(current_h))
        detail[rel] = {"status": "passed" if ok else "tampered", "hash": current_h}
        if not ok:
            passed = False
    return passed, detail
