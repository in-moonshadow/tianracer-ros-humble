#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 取消导航目标工具。
# MoveBaseCancel：取消 move_base 所有目标 -> Nav2 的 <action>/_action/cancel_goal 服务。
# 对外接口：wait_for_subscribers / run_timer。
#
# 注意：rclpy 的 ActionClient 既没有 cancel_all_goals_async 也没有 cancel_goal_async，
# 旧写法每次调用都抛 AttributeError。取消「全部目标」的正确做法是调用 action 的隐藏
# 服务 <action>/_action/cancel_goal，请求里 goal_id 传全零 UUID。

import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal


class MoveBaseCancel(Node):
    """取消 Nav2 全部导航目标（等价于 ROS1 的 cancel_all_goals）。"""

    def __init__(self):
        super().__init__('move_base_canceler')
        ns = self.get_namespace().rstrip('/')
        action = f'{ns}/navigate_to_pose' if ns else '/navigate_to_pose'
        self._client = self.create_client(CancelGoal, action + '/_action/cancel_goal')
        self._cancel_timer = self.create_timer(2.0, self.run_timer)
        self.get_logger().info(
            'MoveBaseCancel: waiting for navigate_to_pose cancel service...')

    def wait_for_subscribers(self, timeout=10.0):
        """等待 action 的 cancel 服务上线。"""
        return self._client.wait_for_service(timeout_sec=timeout)

    def cancel_all_goals(self):
        """取消所有进行中的导航目标（goal_info 默认全零 UUID = 取消全部）。"""
        self.get_logger().info('trying to cancel all goals...')
        future = self._client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        resp = future.result()
        if resp is None:
            self.get_logger().warn('cancel failed: no response')
        elif resp.return_code != CancelGoal.Response.ERROR_NONE:
            self.get_logger().warn('cancel failed (return_code=%s)' % resp.return_code)
        else:
            self.get_logger().info(
                '%d goals have been canceled' % len(resp.goals_canceling))

    def run_timer(self):
        """定时取消目标（reset 场景用）。"""
        if self.wait_for_subscribers(timeout=0.5):
            self.cancel_all_goals()


def main(args=None):
    rclpy.init(args=args)
    node = MoveBaseCancel()
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
