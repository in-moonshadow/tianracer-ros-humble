#!/usr/bin/env bash
# =============================================================================
# TEB 赛程测试启动器（一轮 = 起 gz+nav2+裁判 → /start → 等结束 → 事件计数 → 清理）
# =============================================================================
# 用法：
#   bash tools/teb_round.sh <轮次标签>                # 例：bash tools/teb_round.sh kine1
#   OUT=/path/to/logs bash tools/teb_round.sh kine1   # 换输出目录（默认 /tmp/tebfinal/<标签>）
#   WS=/path/to/ws    bash tools/teb_round.sh kine1   # 换工作区（默认取本仓库根）
# 配套工具：teb_sampler.py（采样，本脚本自动起）、teb_analyze.py（出报告）、
#           teb_locate.py（定位卡点）。分析：python3 tools/teb_analyze.py <轮次目录>
#
# 本脚本内置三条本项目踩过多次的防护，改它之前先读：
#   1. 前置残留检查：上一轮进程未回收则**拒绝开跑**（不能用 pkill -f，会自匹配误杀自己）。
#   2. /dev/shm 孤儿 FastRTPS 段清理：跨轮累积到上百段会让 Nav2 bringup 的 DDS 发现超时
#      卡死（日志特征 failed to send response to /bt_navigator/get_state (timeout)）。
#   3. 就绪等待用**墙钟 150 s 上限**：DDS 卡死时旧逻辑会空转十几分钟，并且带着一个死掉的
#      bt_navigator 硬跑一轮（必然废数据）。
# 注意：source ROS setup.bash 时不能开 set -u（AMENT_TRACE_SETUP_FILES 未绑定会中止）。
set -o pipefail
TOOLS_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS="${WS:-$(cd "$TOOLS_DIR/../.." && pwd)}"
R="${1:-r1}"
LOG="${OUT:-/tmp/tebfinal}/$R"
mkdir -p "$LOG"
PIDF=$LOG/pids.txt
: > "$PIDF"

# 注意：source ROS setup.bash 时不能开 set -u（AMENT_TRACE_SETUP_FILES 未绑定会中止）
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# 前置残留检查：上一轮的进程必须全部回收，否则本轮拒绝开始。
# 注意不能用 `pgrep/pkill -f`：本脚本自身的命令行里就含有这些关键字，
# 会自匹配（本项目已因此误杀过自己的 shell 多次）。这里用 ps 输出 + 显式排除自身。
echo "[$R] ===== 前置残留检查 ====="
resid=$(ps -eo pid,cmd --no-headers | awk '
  /snapshot-bash|teb_round\.sh|tebfinal_round\.sh|ros2cli\.daemon|ros2-daemon|awk |ps -eo/ {next}
  /ign gazebo|ros2 launch|nav2_[a-z]|judge_system|judge_display|servo_commands|transform\.py|teb_sampler|robot_state_publisher|parameter_bridge/ {print "  " $0}
')
if [ -n "$resid" ]; then
  echo "[$R] 检测到上一轮残留进程，拒绝开始本轮："
  echo "$resid"
  echo "[$R] 请先按显式 PID 清理后重跑"
  exit 2
fi
echo "[$R] 无残留，继续"

# 清理**孤儿** FastRTPS 共享内存段：收尾用 kill -KILL 时被强杀的 ros2 进程不会自 unlink，
# 段会跨轮累积（2026-09-13 实测累积到 108~136 段时，Nav2 bringup 连续两轮 DDS 发现超时卡死）。
# 只删「lsof 查不到持有者」的段，ros2 daemon 持有的 port*/_el 段一律保留。
held_shm=$(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $NF}' | sort -u)
shm_before=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
for sf in /dev/shm/fastrtps_*; do
  [ -e "$sf" ] || continue
  case "$held_shm" in *"$sf"*) ;; *) rm -f "$sf" 2>/dev/null ;; esac
done
shm_after=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
echo "[$R] /dev/shm 孤儿段清理: $shm_before -> $shm_after 个（被持有的一律保留）"

echo "[$R] ===== 启动 gz + control ====="
nohup ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py gui:=false \
  > "$LOG/gz.log" 2>&1 &
echo $! >> "$PIDF"

for i in $(seq 1 60); do
  if timeout 5 ros2 topic echo --once /clock >/dev/null 2>&1; then echo "[$R] /clock OK"; break; fi
  sleep 2
done

echo "[$R] ===== 启动 nav2 (navfn_teb) ====="
nohup ros2 launch tianracer_navigation2 nav2.launch.py use_sim_time:=true use_rviz:=false use_planner:=navfn_teb \
  > "$LOG/nav2.log" 2>&1 &
echo $! >> "$PIDF"

# 就绪等待改为**墙钟 150s 上限**。原写法 `for i in $(seq 1 90)` 每轮要跑两次带 5s 超时的
# ros2 查询，实际可拖到十几分钟；而 DDS 响应丢失卡死（bt_navigator 永不 active，
# 日志有 "failed to send response to /bt_navigator/get_state (timeout)"）时判据永远不会过，
# 旧逻辑超时后还会带着一个死的 bt_navigator 硬跑一轮（必然废数据）。
# 现在：150s 内未就绪即判卡死 → ok=0 → 跳过赛程直接清理（下同）。
ok=0
deadline=$(( $(date +%s) + 150 ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  n=$(grep -c "Managed nodes are active" "$LOG/nav2.log" 2>/dev/null); n=${n:-0}
  lc=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
  pubs=$(timeout 5 ros2 topic info /navigate_to_pose/_action/status 2>/dev/null | grep -i "Publisher count" | grep -o "[0-9]\+" | head -1); pubs=${pubs:-0}
  if [ "$n" -ge 2 ] && echo "$lc" | grep -q active && [ "$pubs" -ge 1 ]; then
    echo "[$R] Nav2 就绪: managed=$n lifecycle=$lc pubs=$pubs"; ok=1; break
  fi
  sleep 3
done
if [ "$ok" != 1 ]; then
  echo "[$R] Nav2 150s 内未就绪（managed=$n lifecycle=$lc pubs=$pubs）→ 判定 DDS 响应丢失卡死，跳过本轮"
fi

if [ "$ok" = 1 ]; then
echo "[$R] ===== 确认加载 TEB ====="
grep -m3 -E "Created controller|TebLocalPlannerROS|Footprint model" "$LOG/nav2.log" | head -3

echo "[$R] ===== 参数实际加载值 ====="
{
  for k in max_vel_theta max_vel_x max_vel_x_backwards min_obstacle_dist weight_obstacle cmd_angle_instead_rotvel min_turning_radius weight_kinematics_turning_radius weight_optimaltime; do
    printf 'FollowPath.%-24s = ' "$k"
    timeout 5 ros2 param get /controller_server "FollowPath.$k" 2>/dev/null | tail -1
  done
  printf 'smoother max_velocity%-15s = ' ''
  timeout 5 ros2 param get /velocity_smoother max_velocity 2>/dev/null | tail -1
} | tee "$LOG/params_loaded.txt"

echo "[$R] ===== 启动裁判 ====="
nohup ros2 launch tianracer_gazebo judge.launch.py > "$LOG/judge.log" 2>&1 &
echo $! >> "$PIDF"
sleep 6

echo "[$R] ===== 先起采样器（早于 /start）====="
nohup python3 "$TOOLS_DIR/teb_sampler.py" "$LOG/samples.csv" > "$LOG/sampler.log" 2>&1 &
echo $! >> "$PIDF"
sleep 5

echo "[$R] ===== /start @ $(date +%s.%N) ====="
# 与 dwb_round.sh 同口径：用常驻 rclpy client（judge_start.py）发 /start，
# 而非 `ros2 service call` CLI —— 后者每次新起 Python 进程（0.30~0.36s 起），
# 该开销会被算进「起步死时间」，而真实用户点 judge_display.py 的按钮没有它。
if ! timeout 40 python3 "$TOOLS_DIR/judge_start.py" "$LOG/t_start.txt" > "$LOG/start_call.log" 2>&1; then
  echo "[$R] !! 常驻 client 发 /start 失败，回退到 ros2 service call（口径会偏大）"
  date +%s.%N > "$LOG/t_start.txt"
  timeout 10 ros2 service call /start std_srvs/srv/Empty >> "$LOG/start_call.log" 2>&1
fi
echo "[$R] /start 返回: $(tail -2 "$LOG/start_call.log" | tr '\n' ' ')"

echo "[$R] ===== 等待赛程结束（最多 480s）====="
for i in $(seq 1 160); do
  if grep -qE "FINAL state|has not moved for a long time|No checkpoint detected" "$LOG/judge.log" 2>/dev/null; then
    echo "[$R] 结束信号 @ $(date +%s.%N)"; break
  fi
  sleep 3
done
sleep 3
date +%s.%N > "$LOG/t_end.txt"

echo "[$R] ===== 终局 ====="
grep -E "FINAL state|has not moved|No checkpoint detected|通过第" "$LOG/judge.log" | tail -25

echo "[$R] ===== 事件计数 ====="
for pat in "trajectory is not feasible" "Controller patience exceeded" "possible oscillation" "failed to create plan" "No valid trajectories" "clear entirely the global_costmap" "max_vel_x_backwards"; do
  c=$(grep -c "$pat" "$LOG/nav2.log" 2>/dev/null); c=${c:-0}
  printf '  %-34s %s\n' "$pat" "$c"
done

echo "[$R] ===== 采样计数 ====="
tail -2 "$LOG/sampler.log"

fi   # ← 结束 "if [ $ok = 1 ]" 赛程段（未就绪时整段跳过）

echo "[$R] ===== 清理（显式 PID）====="
# 先 TERM 记录在案的 PID，再按 /proc 复核残留（不按进程名匹配，避免自匹配）
while read -r p; do
  [ -n "$p" ] && kill -TERM "$p" 2>/dev/null && echo "  TERM $p"
done < "$PIDF"
sleep 8
# 复查：仍持有 DDS 共享内存的 PID 逐个 KILL。
# 排除项改为**按 cmdline 动态排除 ros2 daemon**（原来的硬编码 PID 表已过期，
# daemon 重启后 PID 会变，硬编码会导致误杀 daemon → 后续 ros2 CLI 全部失效）。
left=$(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $2}' | sort -u | while read -r p; do
  cl=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null)
  case "$cl" in *ros2cli.daemon*|*ros2-daemon*) ;; *) echo "$p";; esac
done)
if [ -n "$left" ]; then
  echo "  仍存活: $(echo $left | tr '\n' ' ')"
  for p in $left; do kill -KILL "$p" 2>/dev/null && echo "  KILL $p"; done
  sleep 3
fi
echo "[$R] 清理后剩余 DDS 持有者: $(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $2}' | sort -u | tr '\n' ' ')"
echo "[$R] 脚本结束"
