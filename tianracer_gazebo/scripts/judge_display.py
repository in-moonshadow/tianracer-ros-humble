#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 裁判系统监控评分窗口（tkinter），深色遥测仪表盘风格。
#
# 界面文案与交互要素按本项目的评分展示需求自行定义：
#   标题   : "Tianbot 官方监控评分系统"        几何: 1000x400
#   字体   : 数字用 DSEG7Classic（数码管），标签用 Noto Sans CJK SC
#   标签   : ROBOT_NAME / WORLD_NAME / 总分数为 / 用时
#   状态   : 测评系统已加载完成 / 请点击启动按钮开始测评 / 目标代码已启动 /
#            程序运行中 / 刹车中 / 完成全局比赛 / 停车超时，已终止
#            （后三条由 /score_display 的 state 字段驱动，见 STATE_TEXT；
#              重置后按 idle 渲染，分数与用时立即清零，不再停留于「重置中」）
#   按钮   : 启动 / 刹车 / 重置  ->  /start /brake /reset
#   配色   : 深色底 + 青色分数 + 琥珀色用时（见下方 THEME）
#
# 用法：ros2 launch tianracer_gazebo judge.launch.py      （enable_display 默认 true）
#       ros2 run tianracer_gazebo judge_display.py

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TITLE = 'Tianbot 官方监控评分系统'
WIN_W, WIN_H = 1000, 400
# 数字用 DSEG7Classic 数码管字体，标签用 times。
# 实际安装的族名可能带空格（DSEG7 Classic），且 times 常以 Times New Roman 等替代，
# 故给出候选链，按优先级取第一个可用的。
FONT_DIGIT_CANDS = ['DSEG7Classic', 'DSEG7 Classic']
FONT_LABEL_CANDS = ['Noto Sans CJK SC', 'Source Han Sans SC', 'WenQuanYi Zen Hei',
                    'DejaVu Sans']

# 深色遥测仪表盘配色
THEME = {
    'bg': '#0b0f14',                 # 窗口底
    'card': '#151b23',               # 卡片底
    'border': '#232c38',             # 卡片描边
    'divider': '#1c2430',            # 分隔线
    'fg': '#e6edf3',                 # 主文字
    'muted': '#8b949e',              # 次要文字
    'cyan': '#22d3ee',               # 分数（数码管）
    'amber': '#fbbf24',              # 用时（数码管）
    'green': '#22c55e',              # 运行中
    'red': '#ef4444',                # 制动 / 异常
    'btn': '#1c2430',
    'btn_hover': '#28323f',
    'btn_primary': '#0e7490',
    'btn_primary_hover': '#0891b2',
    'btn_danger': '#7f1d1d',
    'btn_danger_hover': '#991b1b',
}

# /score_display 的 state 字段 -> 状态行渲染（status 文案, THEME 配色键, 提示行文案）。
# 完赛状态 'finished' 对应的界面文案为「完成全局比赛」。
# 未携带 state（旧版发布者）时按 running 兜底。
STATE_TEXT = {
    'idle':     ('请点击启动按钮开始测评', 'amber', '计时与分数已清零，请点击启动按钮开始测评'),
    'running':  ('程序运行中', 'green', '程序运行中'),
    'finished': ('完成全局比赛', 'cyan', '已完赛，如需重新比赛请点击重置'),
    'stopped':  ('停车超时，已终止', 'red', '已停表，如需重新比赛请点击重置'),
}


class JudgeDisplay(Node):
    """订阅 /score_display，并把裁判服务（start/brake/reset）暴露给界面按钮。"""

    def __init__(self):
        super().__init__('judge_display')
        self.declare_parameter('topic', 'score_display')
        self.declare_parameter('world', '')
        self.declare_parameter('robot_name', '')
        self.declare_parameter('poll_ms', 100)

        self._raw = ''
        self._fields = {}

        # 与 judge_system.py 同一套绝对话题名规则，保证命名空间一致时能对上
        ns = os.getenv("TIANBOT_NAME", os.getenv("TIANRACER_NAME", ""))
        if ns in ("", "/"):
            ns = ""

        def topic_of(name):
            return f'/{ns}/{name}' if ns else f'/{name}'

        topic = self.get_parameter('topic').value
        if not topic.startswith('/'):
            topic = topic_of(topic)
        self._sub = self.create_subscription(String, topic, self._on_score, 1)

        # 注意：不能叫 _clients / _services —— 那是 rclpy Node 内部列表，覆盖会让 create_client 崩溃。
        self._srv_clients = {}
        for name in ('start', 'brake', 'reset'):
            try:
                from std_srvs.srv import Empty
                self._srv_clients[name] = self.create_client(Empty, topic_of(name))
            except Exception as exc:
                self.get_logger().warn(f'服务 {name} 不可用: {exc}')

    def _on_score(self, msg):
        # 宽容解析 "k: v | k: v"：判分格式若调整，这里按实际收到的键渲染，不会崩。
        fields = {}
        for part in msg.data.split('|'):
            if ':' in part:
                key, val = part.split(':', 1)
                fields[key.strip()] = val.strip()
        self._raw = msg.data
        self._fields = fields

    def snapshot(self):
        return self._raw, self._fields

    def call(self, name):
        """按钮回调：异步调用裁判服务（由主循环的 spin_once 推进）。"""
        client = self._srv_clients.get(name)
        if client is None:
            return '服务不可用'
        if not client.service_is_ready():
            return '裁判系统未就绪'
        client.call_async(client.srv_type.Request())
        return None


def _norm_family(name):
    # 字体族名可能带空格（DSEG7 Classic 实际安装为带空格形式），比较前归一化。
    return name.replace(' ', '').lower()


def _pick_font(families, wanted, fallback):
    """按 wanted 优先级取第一个可用的族名；都没有则返回 fallback。"""
    by_norm = {}
    for fam in families:
        by_norm.setdefault(_norm_family(fam), fam)
    for want in wanted:
        hit = by_norm.get(_norm_family(want))
        if hit:
            return hit
    return fallback


def _build_ui(node, root, tk, tkfont, digit_family, label_family, world, robot):
    """构建界面并把控件/回调收进一个字典返回（与事件循环分离，便于自检与测试）。"""
    T = THEME
    root.title(TITLE)
    root.geometry(f'{WIN_W}x{WIN_H}')
    root.resizable(False, False)
    root.configure(bg=T['bg'])

    # 用像素字号（负数）而非点数：CJK 字体（Noto Sans CJK）按点数计时 linespace 极大
    # （12pt 就有 48px），会把布局撑爆；DSEG7 同理。像素字号下量度可控。
    f_h1 = tkfont.Font(family=label_family, size=-20, weight='bold')
    f_label = tkfont.Font(family=label_family, size=-12)
    f_strong = tkfont.Font(family=label_family, size=-14, weight='bold')
    f_caps = tkfont.Font(family=label_family, size=-10)
    f_digit = tkfont.Font(family=digit_family, size=-30, weight='bold')
    f_btn = tkfont.Font(family=label_family, size=-15, weight='bold')

    # ── 顶栏：标题 + 状态指示 ───────────────────────────────────
    header = tk.Frame(root, bg=T['bg'])
    header.pack(side='top', fill='x', padx=28, pady=(14, 8))
    tk.Label(header, text=TITLE, font=f_h1, fg=T['fg'],
             bg=T['bg']).pack(side='left')
    status_var = tk.StringVar(value='测评系统已加载完成')
    status_lbl = tk.Label(header, textvariable=status_var, font=f_strong,
                          fg=T['green'], bg=T['bg'])
    status_lbl.pack(side='right')

    tk.Frame(root, bg=T['divider'], height=1).pack(side='top', fill='x', padx=28)

    # 按钮条先 pack（side='bottom'）占住底部空间，否则会被上方内容挤出窗口。
    btns = tk.Frame(root, bg=T['bg'])
    btns.pack(side='bottom', pady=14)

    # ── 元信息：ROBOT_NAME / WORLD_NAME ────────────────────────
    meta = tk.Frame(root, bg=T['bg'])
    meta.pack(side='top', fill='x', padx=28, pady=(8, 0))

    def meta_item(parent, caps, value):
        box = tk.Frame(parent, bg=T['bg'])
        tk.Label(box, text=caps, font=f_caps, fg=T['muted'],
                 bg=T['bg']).pack(anchor='w')
        tk.Label(box, text=value, font=f_strong, fg=T['fg'],
                 bg=T['bg']).pack(anchor='w')
        return box

    meta_item(meta, 'ROBOT_NAME', robot).pack(side='left')
    meta_item(meta, 'WORLD_NAME', world).pack(side='left', padx=40)

    # ── 两块数码管卡片：总分数 / 用时 ──────────────────────────
    def make_card(parent, caption, color):
        card = tk.Frame(parent, bg=T['card'],
                        highlightbackground=T['border'], highlightthickness=1)
        tk.Label(card, text=caption, font=f_label, fg=T['muted'],
                 bg=T['card']).pack(side='top', padx=34, pady=(8, 0))
        var = tk.StringVar(value='--')
        val = tk.Label(card, textvariable=var, font=f_digit, fg=color,
                       bg=T['card'])
        val.pack(side='top', padx=34, pady=(2, 8))
        return card, var, val

    cards = tk.Frame(root, bg=T['bg'])
    cards.pack(side='top', pady=(10, 4))
    score_card, score_var, score_lbl = make_card(cards, '总分数为', T['cyan'])
    score_card.pack(side='left', padx=20)
    el_card, elapsed_var, elapsed_lbl = make_card(cards, '用时', T['amber'])
    elapsed_var.set('00:00:00')
    el_card.pack(side='left', padx=20)

    # ── 提示行 ─────────────────────────────────────────────────
    hint_var = tk.StringVar(value='请点击启动按钮开始测评')
    hint_lbl = tk.Label(root, textvariable=hint_var, font=f_label,
                        fg=T['muted'], bg=T['bg'])
    hint_lbl.pack(side='top', pady=(2, 0))

    # ── 按钮：启动 / 刹车 / 重置 ────────────────────────────────
    state = {'braking': False, 'raw': None}

    def set_status(text, color):
        status_var.set(text)
        status_lbl.config(fg=color)

    def make_button(parent, text, bg, hover, fg='#ffffff', command=None):
        b = tk.Button(parent, text=text, font=f_btn, bg=bg, fg=fg,
                      activebackground=hover, activeforeground=fg,
                      relief='flat', bd=0, highlightthickness=0,
                      padx=30, pady=10, cursor='hand2', command=command)
        b.bind('<Enter>', lambda _e: b.config(bg=hover))
        b.bind('<Leave>', lambda _e: b.config(bg=bg))
        return b

    def on_start():
        err = node.call('start')
        if err:
            set_status(err, T['red'])
        else:
            set_status('目标代码已启动', T['green'])
            hint_var.set('程序运行中')

    def on_brake():
        err = node.call('brake')
        if err:
            set_status(err, T['red'])
            return
        state['braking'] = not state['braking']
        if state['braking']:
            set_status('刹车中', T['red'])
            hint_var.set('再次点击刹车可解除制动')
        else:
            set_status('程序运行中', T['green'])
            hint_var.set('程序运行中')

    def on_reset():
        err = node.call('reset')
        state['braking'] = False
        if err:
            set_status(err, T['red'])
            return
        # 立即清零，不等裁判那条 idle 载荷回来：重置与清零之间不该有可见的中间态。
        score_var.set('0.000')
        elapsed_var.set('00:00:00')
        text, color, hint = STATE_TEXT['idle']
        set_status(text, THEME[color])
        hint_var.set(hint)

    make_button(btns, '启动', T['btn_primary'], T['btn_primary_hover'],
                command=on_start).pack(side='left', padx=10)
    make_button(btns, '刹车', T['btn_danger'], T['btn_danger_hover'],
                command=on_brake).pack(side='left', padx=10)
    make_button(btns, '重置', T['btn'], T['btn_hover'], fg=T['fg'],
                command=on_reset).pack(side='left', padx=10)

    return {
        'root': root, 'btns': btns, 'cards': cards,
        'score_var': score_var, 'elapsed_var': elapsed_var,
        'score_lbl': score_lbl, 'elapsed_lbl': elapsed_lbl,
        'status_var': status_var, 'status_lbl': status_lbl,
        'hint_var': hint_var, 'hint_lbl': hint_lbl,
        'set_status': set_status, 'state': state,
        'poll_ms': node.get_parameter('poll_ms').value,
    }


def _make_window(node):
    """建窗口；无显示环境时返回 None。"""
    try:
        import tkinter as tk
        import tkinter.font as tkfont
    except Exception as exc:
        node.get_logger().error(f'tkinter 不可用，跳过监控窗口: {exc}')
        return None
    try:
        root = tk.Tk()
    except Exception as exc:
        node.get_logger().warn(f'无法创建窗口（无显示环境？）: {exc}')
        return None

    families = set(tkfont.families(root))
    digit_family = _pick_font(families, FONT_DIGIT_CANDS, 'DejaVu Sans Mono')
    label_family = _pick_font(families, FONT_LABEL_CANDS, 'DejaVu Sans')
    if _norm_family(digit_family) not in [_norm_family(f) for f in FONT_DIGIT_CANDS]:
        node.get_logger().warn(
            f'未找到 DSEG 数码管字体，数字回退为 {digit_family}'
            '（界面为数码管样式；可把 DSEG7-Classic 的 ttf 装到 ~/.fonts）')

    world = node.get_parameter('world').value or '-'
    robot = node.get_parameter('robot_name').value or '-'
    ui = _build_ui(node, root, tk, tkfont, digit_family, label_family, world, robot)

    score_var, elapsed_var = ui['score_var'], ui['elapsed_var']
    hint_var, state, set_status = ui['hint_var'], ui['state'], ui['set_status']

    def pump():
        # rclpy 与 tkinter 同线程：用 after() 轮转，避免 tkinter 跨线程不安全。
        if not rclpy.ok():
            root.destroy()
            return
        rclpy.spin_once(node, timeout_sec=0.0)
        raw, fields = node.snapshot()
        if raw != state['raw']:
            state['raw'] = raw
            if fields:
                score_var.set(fields.get('score', '--'))
                secs = fields.get('elapsed', '0s').rstrip('s')
                try:
                    total = int(float(secs))
                    elapsed_var.set(
                        f'{total // 3600:02d}:{total % 3600 // 60:02d}:{total % 60:02d}')
                except ValueError:
                    elapsed_var.set('00:00:00')
                if not state['braking']:
                    text, color, hint = STATE_TEXT.get(
                        fields.get('state', ''), STATE_TEXT['running'])
                    set_status(text, THEME[color])
                    hint_var.set(hint)
            else:
                score_var.set('--')
        root.after(ui['poll_ms'], pump)

    # 关闭窗口即退出节点（绑定 WM_DELETE_WINDOW）
    root.protocol('WM_DELETE_WINDOW', root.destroy)
    root.after(0, pump)
    root.mainloop()
    return root


class _StubNode:
    """自检用：不依赖 ROS 也能构建界面。"""

    def __init__(self, world='-', robot='-', poll_ms=100):
        self._p = {'world': world, 'robot_name': robot, 'poll_ms': poll_ms}

    class _Logger:
        def warn(self, m):
            print(f'[WARN] {m}')

        def error(self, m):
            print(f'[ERROR] {m}')

        info = warn

    def get_logger(self):
        return self._Logger()

    def get_parameter(self, name):
        holder = type('P', (), {})()
        holder.value = self._p.get(name)
        return holder

    def snapshot(self):
        return '', {}

    def call(self, name):
        return 'selftest'


def _selftest():
    """JUDGE_DISPLAY_SELFTEST=1：只建界面并打印各控件实际尺寸，用于快速排版核对。"""
    import tkinter as tk
    import tkinter.font as tkfont
    root = tk.Tk()
    families = set(tkfont.families(root))
    digit = _pick_font(families, FONT_DIGIT_CANDS, 'DejaVu Sans Mono')
    label = _pick_font(families, FONT_LABEL_CANDS, 'DejaVu Sans')
    print(f'font: digit={digit}  label={label}')
    ui = _build_ui(_StubNode(), root, tk, tkfont, digit, label,
                   'tianracer_racetrack', 'tianracer')
    root.update_idletasks()
    root.update()
    print(f'root: {root.winfo_width()}x{root.winfo_height()} (目标 {WIN_W}x{WIN_H})')
    for key in ('cards', 'score_lbl', 'elapsed_lbl', 'hint_lbl', 'btns'):
        wdg = ui.get(key)
        if wdg is not None:
            print(f'  {key:12s} y={wdg.winfo_y():4d}  h={wdg.winfo_height():3d}'
                  f'  reqh={wdg.winfo_reqheight():3d}  mapped={bool(wdg.winfo_ismapped())}')
    root.destroy()


def main(args=None):
    if os.environ.get('JUDGE_DISPLAY_SELFTEST'):
        _selftest()
        return
    rclpy.init(args=args)
    node = JudgeDisplay()
    try:
        _make_window(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
