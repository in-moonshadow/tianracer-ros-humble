#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 赛道检查门（door）RViz 可视化节点。
#
# 目的：把 <world>_check_points.yaml 的 3 个检查门画到 RViz，并把「当前待通过的
# 门」实时高亮，便于肉眼对照减速/走线与门的位置关系。
#
# 门 = 线段：与裁判 (position_check.load_checkpoint) 完全相同的规则——
#   check_points.yaml 的 N 个点两两配对（下标偶数开新线段、奇数补全），每 2 点成 1 门。
# 因此本节点复用 waypoint_race.utils.get_waypoints 读同一文件、按同一配对规则切分，
# 保证画出来的门与裁判判定用的几何**逐点一致**。
#
# 实时高亮的真值源：订阅裁判发布的 /<ns>/score_display（文本 "key: val | key: val"），
# 取其中的 points: N/M 字段。N = 已通过门数，故「当前待通过门」= N % 3，与裁判
# position_check.analysis 内部用的 ana_cnt % 3 同一语义。不重复实现门的判定逻辑。
#
# 三色语义：
#   暗灰 = 非当前目标；  亮黄 = 当前待通过；  绿色 = 已通过（本圈内已完成）
# 注：每圈 3 门循环，故「已通过」只表示本轮（本圈）已压过，跨圈会重置为待通过。
#
# 用法：ros2 run tianracer_gazebo door_markers.py
#       ros2 launch tianracer_gazebo door_markers.launch.py

import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

# 使同目录脚本与 waypoint_race 子包可被 import（ament 安装后脚本位于 lib/<pkg>/ 下）
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import waypoint_race.utils as utils

DOORS_PER_LAP = 3          # 与 judge_system.DOORS_PER_LAP 一致（每圈 3 门）

LINE_WIDTH = 0.06          # 门线粗细（m）
LINE_HEIGHT = 1.0          # 门线高度（m），做成竖直薄墙便于 3D 观察
LABEL_SIZE = 0.45          # 编号文字高度（m）
LABEL_Z = 0.9              # 编号离地高度（m）

# 三色：暗灰（非目标）/ 亮黄（当前目标）/ 绿（本圈已过）
COLOR_IDLE = (0.45, 0.45, 0.50, 0.85)
COLOR_ACTIVE = (1.00, 0.82, 0.10, 1.00)
COLOR_PASSED = (0.15, 0.85, 0.25, 0.95)

# ── 场景内图例（画在 RViz 三维空间里，不是屏幕浮层）────────────────
# 位置：赛道**下方**的空白区。赛道占用框为 x∈[-5.47,2.35] y∈[-9.10,5.18]
# 图例框位置按地图占用像素确定，约 x∈[-5.6,-2.6] y∈[-11.0,-9.7]，
# 与赛道不重叠；该处地图值为「未知」(205)，不会与门/代价地图混淆。
# 用 map 坐标系，故随视角缩放/平移保持与门一致的相对位置。
# 可用参数 legend:=false 关闭。
# ⚠️ 文案用英文：RViz 的 TEXT_VIEW_FACING 走 OGRE 位图字体，中文会渲染成方块。
LEGEND_ORIGIN = (-5.6, -9.7)     # 图例左上角锚点 (x, y)，单位 m
LEGEND_LINE_LEN = 0.55           # 图例色条长度 (m)
LEGEND_ROW_GAP = 0.42            # 行间距 (m)
LEGEND_TEXT_SIZE = 0.26          # 图例文字高度 (m)
LEGEND_TITLE_SIZE = 0.30
# 图例条目：(显示文案, 颜色)
LEGEND_ITEMS = [
    ('1 / 2 / 3  door index', None),          # None = 标题行（用主文字色）
    ('passed this lap', COLOR_PASSED),
    ('current target', COLOR_ACTIVE),
    ('not yet', COLOR_IDLE),
]


class DoorMarkers(Node):
    def __init__(self):
        super().__init__('door_markers')

        ns = os.getenv('TIANBOT_NAME', os.getenv('TIANRACER_NAME', ''))
        self._ns = ns if ns not in ('', '/') else ''
        self._prefix = f'/{self._ns}' if self._ns else ''

        self.declare_parameter('world', os.getenv('TIANRACER_WORLD', 'tianracer_racetrack'))
        # frame_id 跟随命名空间：无 TIANBOT_NAME 时 TF 树就是 map，有名字时是 <ns>/map。
        # 默认值 'auto' 表示按 _ns 推导；显式传值可覆盖。
        self.declare_parameter('frame_id', 'auto')
        self.declare_parameter('publish_period', 0.5)   # 秒；MarkerArray 需周期发布，
        # 因为 RViz 的 transient_local 支持依版本而异，周期发布最稳
        self.declare_parameter('legend', True)          # 是否在场景内画门图例

        frame = self.get_parameter('frame_id').value
        self._frame_id = (f'{self._ns}/map' if self._ns else 'map') if frame == 'auto' else frame
        self._passed = 0                 # 已通过门数（来自 score_display 的 points: N/M）
        self._cleared = False            # 是否已发过 DELETEALL（只发一次，避免闪烁）
        self._doors = self._load_doors(self.get_parameter('world').value)

        if not self._doors:
            self.get_logger().error('未读到任何检查门，节点将只做空发布')

        # 话题：与裁判同一命名空间约定
        self._pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/door_markers', 1)
        self.create_subscription(
            String, f'{self._prefix}/score_display', self._on_score, 1)

        period = float(self.get_parameter('publish_period').value)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            'door_markers ready: %d 门, world=%s, frame=%s'
            % (len(self._doors), self.get_parameter('world').value, self._frame_id))

    def _load_doors(self, world):
        """读 <world>_check_points.yaml，按裁判同样的两两配对规则切成门线段。"""
        try:
            pkg = get_package_share_directory('tianracer_gazebo')
        except Exception as e:
            self.get_logger().error('找不到 tianracer_gazebo: %s' % e)
            return []
        path = os.path.join(pkg, 'waypoint_race', f'{world}_check_points.yaml')
        if not os.path.isfile(path):
            self.get_logger().error('check_points 文件不存在: %s' % path)
            return []

        waypoints = utils.get_waypoints(path)
        doors = []
        pair = None
        for i, w in enumerate(waypoints):
            pose = utils.create_geometry_pose(w)
            pt = (pose.position.x, pose.position.y)
            if i % 2 == 0:
                pair = (pt,)
            else:
                pair = pair + (pt,)
                doors.append(pair)
                pair = None
        if pair is not None:
            # 与 position_check.load_checkpoint 同语义：奇数个点末尾那个不参与配对。
            # 本文件的 tianracer_racetrack_check_points.yaml 就是 7 点 -> 只成 3 门。
            self.get_logger().warn(
                'check_points 有 %d 个点（奇数），最后一个点不参与配对、不会显示'
                % len(waypoints))
        return doors

    def _on_score(self, msg):
        """解析 points: N/M，更新已通过门数。"""
        for part in msg.data.split('|'):
            if ':' not in part:
                continue
            key, val = part.split(':', 1)
            if key.strip() == 'points':
                num = val.strip().split('/')[0]
                try:
                    self._passed = int(num)
                except ValueError:
                    pass
                return

    def _publish(self):
        arr = MarkerArray()

        # DELETEALL 只在首次发布时发一次：每周期重发会让 RViz 反复清空再重画而闪烁。
        if not self._cleared:
            clear = Marker()
            clear.header.frame_id = self._frame_id
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            self._cleared = True

        if not self._doors:
            self._pub.publish(arr)
            return

        # N = 已通过门数；N % 3 既表示「本圈已完成几个门」，
        # 也正好是「当前待通过门的下标」（判定内部即 ana_cnt % 3 顺序推进）。
        done_this_lap = self._passed % DOORS_PER_LAP

        for i, (a, b) in enumerate(self._doors):
            if i < done_this_lap:
                color = COLOR_PASSED          # 本圈已通过
            elif i == done_this_lap:
                color = COLOR_ACTIVE          # 当前待通过
            else:
                color = COLOR_IDLE            # 待通过

            arr.markers.append(self._line(i, a, b, color))
            arr.markers.append(self._label(i, a, b, i + 1, color))

        if self.get_parameter('legend').value:
            arr.markers.extend(self._legend())

        self._pub.publish(arr)

    def _legend(self):
        """场景内图例：一行标题 + 三行色块与说明。

        用 map 坐标系排在赛道外的空白区（见 LEGEND_ORIGIN 注释），故与门同一变换链，
        视角缩放/平移时相对位置保持一致。marker 的 ns='door_legend' 独立，
        便于在 RViz 里单独开关。
        """
        ms = []
        ox, oy = LEGEND_ORIGIN
        mid = LEGEND_LINE_LEN / 2.0
        for row, (text, color) in enumerate(LEGEND_ITEMS):
            z = oy - row * LEGEND_ROW_GAP

            if color is None:
                # 标题行：左侧画一个小方块 + 数字「1」，示意门上的编号样式
                l = Marker()
                l.header.frame_id = self._frame_id
                l.header.stamp = self.get_clock().now().to_msg()
                l.ns = 'door_legend'
                l.id = 100 + row * 2
                l.type = Marker.TEXT_VIEW_FACING
                l.action = Marker.ADD
                l.pose.position.x = ox + mid
                l.pose.position.y = z
                l.pose.position.z = 0.05
                l.pose.orientation.w = 1.0
                l.scale.z = LEGEND_TITLE_SIZE
                l.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                l.text = '1.'
                ms.append(l)
            else:
                # 色条：贴地薄片（不挡视线），颜色与该状态的门的颜色一致
                m = Marker()
                m.header.frame_id = self._frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'door_legend'
                m.id = 100 + row * 2
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = ox + mid
                m.pose.position.y = z
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                m.scale.x = LEGEND_LINE_LEN
                m.scale.y = 0.07
                m.scale.z = 0.02
                m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
                ms.append(m)

            # 每行右侧：文字
            t = Marker()
            t.header.frame_id = self._frame_id
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = 'door_legend'
            t.id = 100 + row * 2 + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = ox + LEGEND_LINE_LEN + 0.12
            t.pose.position.y = z
            t.pose.position.z = 0.05
            t.pose.orientation.w = 1.0
            t.scale.z = LEGEND_TITLE_SIZE if color is None else LEGEND_TEXT_SIZE
            # 标题用亮白，条目用其对应色（与色条呼应）
            c = (0.92, 0.94, 0.97, 1.0) if color is None else color
            t.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0)
            t.text = text
            ms.append(t)
        return ms

    def _line(self, idx, a, b, color):
        """门线段：用 CUBE 拉伸成竖直薄墙，长度 = 两点间距。"""
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = (dx * dx + dy * dy) ** 0.5
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'doors'
        m.id = idx * 2
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = (a[0] + b[0]) / 2.0
        m.pose.position.y = (a[1] + b[1]) / 2.0
        m.pose.position.z = LINE_HEIGHT / 2.0
        # CUBE 默认沿 x 轴，绕 z 转到线段走向
        m.pose.orientation.z = math.sin(math.atan2(dy, dx) / 2.0)
        m.pose.orientation.w = math.cos(math.atan2(dy, dx) / 2.0)
        m.scale.x = length
        m.scale.y = LINE_WIDTH
        m.scale.z = LINE_HEIGHT
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        return m

    def _label(self, idx, a, b, number, color):
        """门编号（1-based，与裁判日志「通过第 N 个门」对齐）。"""
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'door_labels'
        m.id = idx * 2 + 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = (a[0] + b[0]) / 2.0
        m.pose.position.y = (a[1] + b[1]) / 2.0
        m.pose.position.z = LABEL_Z
        m.pose.orientation.w = 1.0
        m.scale.z = LABEL_SIZE
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        m.text = str(number)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = DoorMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
