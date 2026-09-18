# -*- coding: utf-8 -*-
"""车辆几何参数读取（唯一真理源见 param/vehicle_geometry.yaml）。"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

DEFAULT_BASE = "compact"
_FALLBACK_WHEELBASE = "0.261"


def _geometry_path():
    return os.path.join(
        get_package_share_directory("tianracer_description"),
        "param", "vehicle_geometry.yaml")


def load_vehicle_geometry():
    """读取车辆几何表，返回 {车型: {wheelbase, track_width, wheel_radius}}。"""
    with open(_geometry_path(), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["vehicle"]


def get_wheelbase(base=None):
    """按车型返回轴距（字符串，可直接作为 launch 参数默认值）。

    base 为 None 时读环境变量 TIANRACER_BASE；未设置或不可识别时回退 compact。
    """
    if base is None:
        base = os.environ.get("TIANRACER_BASE", DEFAULT_BASE)
    geometry = load_vehicle_geometry()
    entry = geometry.get(base) or geometry.get(DEFAULT_BASE)
    if entry is None or entry.get("wheelbase") is None:
        return _FALLBACK_WHEELBASE
    return str(entry["wheelbase"])
