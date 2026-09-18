#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判定一轮 dwb_round.sh 的结果是否「有效」（能否计入 A/B 统计）。

判据（按具体度排序，命中即返回）
--------------------------------
    controllers_inactive  dwb_round.sh 门禁写下的标记文件 controllers_dead。spawner
                          的服务调用 10s 超时 → 重试撞「already loaded」→ FATAL，
                          车轮/转向控制器从未激活。车一步不动，但 Nav2 侧完全正常
                          （goal 被 accept、cmd_vx 照发）⇒ 只看「车没动」识别不出的
                          那一类，靠标记文件区分。
    dds_goal_response     nav2.log 出现 `rclcpp_action]: Failed to send goal response`
                          （DDS 单向传输故障，铁证）。
    goal_never_accepted   points=0 且 move_dis=0.00 且 accepted=0。
    no_motion             points=0 且 move_dis=0.00 但 accepted>0：目标被接受、cmd 照发，
                          底盘却没动。
    cpu_starved           controller_server 的 `Control loop missed its desired rate`
                          超过 MISSED_RATE_THRESHOLD。这是**机器负载**的指纹：健康轮
                          0~5，被挤住的轮 682~5720（2026-09-16 普查 86 轮，阈值取在
                          两者之间的空隙）。这类轮 DWB 的决策是负载的次生结果，
                          与「配置本身不好」无法区分，故不计入统计。
    no_judge_log / no_final_line    裁判无终局（INCOMPLETE，同样需重跑）。

⚠️ 两条易混的 timeout 日志，不要混为一谈：
    致命 → `[bt_navigator.rclcpp_action]: Failed to send goal response ... (timeout)`
    良性 → `[controller_server.rclcpp]: failed to send response to
            /controller_server/get_parameters (timeout)`，来自本脚本 pre-flight 的
            `ros2 param get`；出现过它的 ti_fix_b 照样 9/9 完赛。
  区分点是 rclcpp_action 的 goal response 与 rclcpp 的 service response。
  切勿用宽泛的 "timeout" 做判据。

用法
    python3 dwb_verdict.py <轮次目录>            # 打印一行结论并写 verdict.txt
    python3 dwb_verdict.py <轮次目录> --quiet     # 只写文件不打印

退出码：0 = VALID，3 = INVALID（需重跑），4 = INCOMPLETE（未跑完/无终局）。
批量脚本据此决定是否重跑，见 dwb_round_retry.sh。
"""
import os
import re
import sys

# 只匹配 rclcpp_action 的 goal response（见文件头警告：不要放宽成 "timeout"）
DDS_GOAL_RESPONSE_RE = re.compile(r'rclcpp_action\]:\s*Failed to send goal response')

# 必须限定 `Control loop`：裸的 "missed its desired rate" 会同时命中 planner_server 的
# `Planner loop missed its desired rate`，而全局规划器不按 20Hz 重规划是完全正常的
# （实测历史最大仅 5 次，故不限定也不会误判，但限定后判据语义才准确）。
MISSED_RATE_TOKEN = 'Control loop missed its desired rate'
MISSED_RATE_THRESHOLD = 500

# dwb_round.sh 的控制器激活门禁未通过时写下的标记文件
CONTROLLERS_DEAD_MARKER = 'controllers_dead'

FINAL_RE = re.compile(
    r'FINAL state:\s*(?P<state>\w+)'
    r'.*?points:\s*(?P<points>\d+)/9'
    r'.*?move_dis:\s*(?P<move_dis>[\d.]+)m'
    r'.*?elapsed:\s*(?P<elapsed>[\d.]+)s')
STATS_RE = re.compile(
    r'状态机计数:.*?goal_sent=(?P<goal_sent>\d+).*?'
    r'accepted=(?P<accepted>\d+).*?'
    r'accept_timeout=(?P<accept_timeout>\d+)')

# detail 的完整字段集。所有 classify() 的返回都必须用它构造：
# main() 会无条件读 points / move_dis 并用 %d 格式化，少一个键就抛异常
# （2026-09-16 踩过 —— 早退分支只返回 {'controllers_dead': True}，判定结果写不出来）。
_DETAIL_FIELDS = ('state', 'points', 'move_dis', 'elapsed', 'goal_sent', 'accepted',
                  'accept_timeout', 'nav2_goal_response_fail', 'missed_rate',
                  'controllers_dead')


def _detail(**over):
    """完整字段的 detail 字典；未指定的字段一律为 None。"""
    d = {k: None for k in _DETAIL_FIELDS}
    d.update(nav2_goal_response_fail=0, missed_rate=0, controllers_dead=False)
    d.update(over)
    return d


def _read(path):
    try:
        with open(path, errors='replace') as f:
            return f.read()
    except OSError:
        return None


def classify(round_dir):
    """返回 (verdict, reason, detail)。verdict ∈ {VALID, INVALID, INCOMPLETE}。"""
    nav2 = _read(os.path.join(round_dir, 'nav2.log'))
    judge = _read(os.path.join(round_dir, 'judge.log'))
    ctrl_dead = os.path.exists(os.path.join(round_dir, CONTROLLERS_DEAD_MARKER))

    # 门禁标记优先于「无裁判日志」：这类轮是确定无效（底盘不会动）且可重跑，
    # 不该退化成 INCOMPLETE 而丢掉「为什么无效」。
    if judge is None:
        if ctrl_dead:
            return 'INVALID', 'controllers_inactive', _detail(controllers_dead=True)
        return 'INCOMPLETE', 'no_judge_log', _detail()

    finals = FINAL_RE.findall(judge)
    if not finals:
        # 裁判没出终局：多半是 Nav2 未就绪整段被跳过，或运行中途被打断
        return 'INCOMPLETE', 'no_final_line', _detail(controllers_dead=ctrl_dead)

    state, points, move_dis, elapsed = finals[-1]
    points, move_dis, elapsed = int(points), float(move_dis), float(elapsed)

    # 用 finditer 而非 findall：findall 返回元组，取不到命名组
    last = next(reversed(list(STATS_RE.finditer(judge))), None)
    goal_sent = int(last.group('goal_sent')) if last else None
    accepted = int(last.group('accepted')) if last else None
    accept_timeout = int(last.group('accept_timeout')) if last else None

    detail = _detail(
        state=state, points=points, move_dis=move_dis, elapsed=elapsed,
        goal_sent=goal_sent, accepted=accepted, accept_timeout=accept_timeout,
        nav2_goal_response_fail=len(DDS_GOAL_RESPONSE_RE.findall(nav2)) if nav2 else 0,
        missed_rate=nav2.count(MISSED_RATE_TOKEN) if nav2 else 0,
        controllers_dead=ctrl_dead)

    if ctrl_dead:                                   # 判据 0：底盘不会动
        return 'INVALID', 'controllers_inactive', detail
    if nav2 and DDS_GOAL_RESPONSE_RE.search(nav2):  # 判据 1：铁证
        return 'INVALID', 'dds_goal_response', detail

    # 判据 2：车一步未动。无论成因都不可计入统计，按 accepted 区分成因。
    if points == 0 and move_dis == 0.0:
        if goal_sent and accepted == 0:
            return 'INVALID', 'goal_never_accepted', detail
        return 'INVALID', 'no_motion', detail

    if detail['missed_rate'] > MISSED_RATE_THRESHOLD:   # 判据 3：机器超售
        return 'INVALID', 'cpu_starved', detail

    return 'VALID', 'ran', detail


def _summary(verdict, reason, d):
    """一行的判定摘要。无终局时 points 等为 None，不能按 %d 格式化。"""
    if verdict == 'INCOMPLETE' or d['points'] is None:
        line = 'VERDICT=%s reason=%s' % (verdict, reason)
        if d['controllers_dead']:
            line += ' controllers_dead=yes'
        return line
    return ('VERDICT=%s reason=%s points=%d/9 move_dis=%.2fm elapsed=%.1fs '
            'goal_sent=%s accepted=%s accept_timeout=%s missed=%d'
            % (verdict, reason, d['points'], d['move_dis'], d['elapsed'],
               d['goal_sent'], d['accepted'], d['accept_timeout'], d['missed_rate']))


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    round_dir = sys.argv[1].rstrip('/')
    if not os.path.isdir(round_dir):
        # 否则写 verdict.txt 时才抛 FileNotFoundError，栈里看不出是「目录名打错了」
        print('轮次目录不存在: %s' % round_dir, file=sys.stderr)
        return 2
    verdict, reason, d = classify(round_dir)

    line = _summary(verdict, reason, d)
    with open(os.path.join(round_dir, 'verdict.txt'), 'w') as f:
        f.write(line + '\n')
    if '--quiet' not in sys.argv:
        print(line)
    return {'VALID': 0, 'INVALID': 3, 'INCOMPLETE': 4}[verdict]


if __name__ == '__main__':
    sys.exit(main())
