#!/usr/bin/env bash
# =============================================================================
# SLAM 建图巡游：起 gz → slam_toolbox(替代 AMCL) → Nav2 → 依次前往路点 → 存图
# =============================================================================
# 用途：重建某个赛道的静态地图，把原图里的 unknown 空洞补上。
#
# 背景（为什么需要它）：
#   `test_indoor` 的地图里，赛道走廊内存大片 unknown(205) 空洞，
#   而本栈 planner_server.GridBased 用 allow_unknown: false + track_unknown_space: true，
#   未知区被当作障碍 → 车跑进空洞后规划永久失败、被停车判据终止（实测 2/9 门卡死）。
#   这是地图覆盖不足，不能靠调 Nav2 参数解决，必须重新建图。
#
# 与 dwb_round.sh 的差别：
#   · 用 nav2_bringup/slam_launch.py（slam_toolbox 发 map->odom）**替代** AMCL+map_server，
#     两者都发 map->odom，不能同时跑。
#   · 不跑裁判（judge），改为按固定路点顺序巡游，走完即存图。
#
# 用法：
#   WORLD=test_indoor bash tools/mapping_round.sh <标签>
#   OUT=/path bash tools/mapping_round.sh <标签>      # 换日志目录
#   产出：/tmp/dwbmapping/<标签>/slam_map.{pgm,yaml}（再手动拷回 maps/）
#
# 三条防护沿用 dwb_round.sh（本项目踩过多次，勿删）：
#   1. 前置残留检查：非空即拒绝开跑（不能用 pkill -f，会自匹配误杀自己）。
#   2. /dev/shm 孤儿 FastRTPS 段清理。
#   3. 就绪等待墙钟上限，避免 DDS 卡死时空转。
set -o pipefail
TOOLS_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS="${WS:-$(cd "$TOOLS_DIR/../.." && pwd)}"
R="${1:-map1}"
LOG="${OUT:-/tmp/dwbmapping}/$R"
mkdir -p "$LOG"
PIDF=$LOG/pids.txt
: > "$PIDF"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

WORLD="${WORLD:-tianracer_racetrack}"
export TIANRACER_WORLD="$WORLD"
if [ ! -f "$WS/tianracer_gazebo/worlds/${WORLD}.world" ]; then
  echo "[$R] !! 找不到 world: worlds/${WORLD}.world"; exit 2
fi
echo "[$R] 建图赛道: ${WORLD}"

# ── 前置残留检查 ─────────────────────────────────────────────────
# ⚠️ 必须排除 awk 自身：awk 的 cmdline 里含下面这些关键字，不排除会**误报自己**
#    （dwb_round.sh 的同一段用 grep -v 兜底，此处等价处理）。
echo "[$R] ===== 前置残留检查 ====="
resid=$(ps -eo pid,cmd --no-headers | awk '
  /snapshot-bash|mapping_round\.sh|ros2cli\.daemon|claude|opencode|awk |grep / {next}
  /ign gazebo|ros2 launch|nav2_[a-z]|slam_toolbox|judge_system|judge_display|servo_commands|transform\.py|dwb_sampler|dwb_recorder|door_markers|robot_state_publisher|parameter_bridge|amcl|map_server/ {print "  " $0}' | head -20)
if [ -n "$resid" ]; then
  echo "[$R] !! 检测到残留进程，拒绝开跑（请先按显式 PID 清理）："
  echo "$resid"
  exit 2
fi
echo "[$R] 无残留，继续"

# ── /dev/shm 孤儿段清理（只删无进程持有的）───────────────────────
shm_before=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
held_shm=$(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $NF}' | sort -u)
for sf in $(ls /dev/shm/fastrtps_* 2>/dev/null); do
  case "$held_shm" in *"$sf"*) ;; *) rm -f "$sf" 2>/dev/null ;; esac
done
shm_after=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
echo "[$R] /dev/shm 孤儿段清理: $shm_before -> $shm_after 个（被持有的一律保留）"

# ── 启动 gz + control ────────────────────────────────────────────
echo "[$R] ===== 启动 gz + control ====="
nohup ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py gui:=false world:="${WORLD}.world" \
  > "$LOG/gz.log" 2>&1 &
echo $! >> "$PIDF"

for i in $(seq 1 60); do
  if timeout 5 ros2 topic echo --once /clock >/dev/null 2>&1; then echo "[$R] /clock OK"; break; fi
  sleep 2
done

# ── 启动 Nav2 导航栈 + slam_toolbox ──────────────────────────────
# ⚠️ nav2_bringup/slam_launch.py **只起 slam_toolbox + map_saver，不含导航栈**
#    （实测：日志只有 lifecycle_manager_slam 与 Managed nodes are active ×1，
#     没有 controller_server/planner_server/bt_navigator → managed=1 pubs=0 判定未就绪）。
#    必须**另外**起 navigation_launch.py 才有 NavigateToPose。
#    两者都吃同一份 navfn_dwb_nav2_params.yaml（含 AMCL 段，但 navigation_launch 不启 AMCL，
#    故不会与 slam_toolbox 争 map->odom）。
echo "[$R] ===== 启动 nav2 导航栈 + slam_toolbox ====="
SLAM_PARAMS="$WS/install/tianracer_slam/share/tianracer_slam/param/mapper_params_online_async.yaml"
NAV_PARAMS="$WS/tianracer_navigation2/params/navfn_dwb_nav2_params.yaml"

# ⚠️ 我们的 params 里有 `<robot_namespace>` 占位符（由 tianracer 自己的 launch 在带 namespace 时替换）。
#    直接喂给 nav2_bringup 的 launch 会解析失败，实测报：
#      Invalid topic name: topic name must not contain characters other than alphanumerics...
#      Lifecycle node local_costmap Caught exception in callback for transition 10 (configure)
#      → managed=0 lifecycle=inactive，导航栈起不来。
#    本脚本不带 namespace（TIANBOT_NAME 为空），故生成一份把 `/` 前缀整体去掉的临时参数文件。
#
# ⚠️ 还要把 slam_toolbox 段**合并进同一份文件**：nav2_bringup/slam_launch.py 用
#    HasNodeParams(params_file, node_name='slam_toolbox') 判断——
#    若参数文件里没有 `slam_toolbox:` 段，它会走「不带参数」分支、**忽略 slam_params_file**，
#    结果是 lifecycle_manager_slam 只配置 map_saver、**从不 configure/activate slam_toolbox**
#    （实测 slam.log 里只有 Configuring map_saver，随后 slam_toolbox 进程退出、map 从未发布
#     → global_costmap 一直 "Received map message is malformed"、车不动、9 个目标全超时）。
#    合并后两个 launch 共用同一份文件，slam_toolbox 段与导航栈段都在。
NAV_PARAMS_NS="$(mktemp /tmp/mapping_ns_params_XXXX.yaml)"
if ! python3 "$TOOLS_DIR/merge_nav_slam_params.py" "$NAV_PARAMS" "$SLAM_PARAMS" "$NAV_PARAMS_NS"; then
  echo "[$R] !! 参数合并失败"; rm -f "$NAV_PARAMS_NS"; exit 2
fi
echo "[$R] 已生成合并参数文件（无占位符 + 含 slam_toolbox 段）: $NAV_PARAMS_NS"

nohup ros2 launch nav2_bringup slam_launch.py \
  use_sim_time:=true autostart:=true use_rviz:=false \
  params_file:="$NAV_PARAMS_NS" \
  > "$LOG/slam.log" 2>&1 &
echo $! >> "$PIDF"
sleep 8
nohup ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true use_rviz:=false \
  params_file:="$NAV_PARAMS_NS" \
  > "$LOG/nav2.log" 2>&1 &
echo $! >> "$PIDF"

ok=0
deadline=$(( $(date +%s) + 150 ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  # 判据：slam 与导航栈**两个** lifecycle_manager 各自报 active
  #（它们分属 slam.log / nav2.log 两个文件，故两处都要 grep；只看一个会永远 managed<2）
  ns=$(grep -c "Managed nodes are active" "$LOG/slam.log" 2>/dev/null); ns=${ns:-0}
  nn=$(grep -c "Managed nodes are active" "$LOG/nav2.log" 2>/dev/null); nn=${nn:-0}
  lc=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
  pubs=$(timeout 5 ros2 topic info /navigate_to_pose/_action/status 2>/dev/null | grep -i "Publisher count" | grep -o "[0-9]\+" | head -1); pubs=${pubs:-0}
  if [ "$ns" -ge 1 ] && [ "$nn" -ge 1 ] && echo "$lc" | grep -q active && [ "$pubs" -ge 1 ]; then
    echo "[$R] 就绪: slam=$ns nav=$nn lifecycle=$lc pubs=$pubs"; ok=1; break
  fi
  sleep 3
done
if [ "$ok" != 1 ]; then
  echo "[$R] !! 150s 内未就绪（slam=$ns nav=$nn lifecycle=$lc pubs=$pubs）→ DDS 卡死，跳过"
fi

# ── 巡游：依次前往路点 + 门端点 ──────────────────────────────────
if [ "$ok" = 1 ]; then
  WP_FILE="$WS/tianracer_gazebo/waypoint_race/${WORLD}_points.yaml"
  CP_FILE="$WS/tianracer_gazebo/waypoint_race/${WORLD}_check_points.yaml"

  # ── 暖机：先原地前进一小段，打破「建图死锁」────────────────────
  # 死锁链（实测）：
  #   slam_toolbox 的 minimum_travel_distance(0.5m) 要求车移动才处理扫描
  #   → 车不走则 /map 恒为 0x0
  #   → global_costmap 判定 "Received map message is malformed. Rejecting."（0x0 视为非法）
  #   → 全局规划 compute_path_to_pose 永久失败
  #   → 车不走 …… 闭环。
  # 破法：绕开导航栈直接发 /cmd_vel 让车走出**一段弧线**（而非纯直行），
  #       使 slam 建出的初始地图有横向宽度 —— 实测纯直行只得到 6x182 的窄条，
  #       目标点 (0.80,-0.14) 落在条外 → worldToMap failed → 仍规划失败。
  #       走弧线后地图能覆盖转向区域，后续导航目标才可能落在图内。
  echo "[$R] ===== 暖机：走弧线建出初始地图（打破建图死锁）====="
  # 直行 + 持续右转，扫出扇形区域
  for _ in $(seq 1 40); do
    timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.30}, angular: {z: -0.45}}" >/dev/null 2>&1
    sleep 0.5
  done
  # 立即刹停，避免撞墙
  timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
  sleep 3
  mw=$(timeout 8 ros2 topic echo --once /map --field info.width 2>/dev/null | grep -oE "[0-9]+" | head -1)
  mh=$(timeout 8 ros2 topic echo --once /map --field info.height 2>/dev/null | grep -oE "[0-9]+" | head -1)
  echo "[$R] 暖机后 /map = ${mw:-0} x ${mh:-0}（两者皆 0 表示 slam 仍未建图）"

  echo "[$R] ===== 巡游（路点 + 门端点，共约 9 点）====="
  timeout 900 python3 "$TOOLS_DIR/mapping_drive.py" "$WP_FILE" "$CP_FILE" "$LOG/drive.log" \
    > "$LOG/drive_stdout.log" 2>&1
  drc=$?
  echo "[$R] 巡游结束（exit=$drc），见 drive.log"
  grep -E "到达|超时|失败|完成" "$LOG/drive.log" 2>/dev/null | tail -20

  # ── 存图 ──────────────────────────────────────────────────────
  echo "[$R] ===== 存图 ====="
  mkdir -p "$LOG/slam_map"
  timeout 120 ros2 run nav2_map_server map_saver_cli -f "$LOG/slam_map/slam_map" \
    --ros-args -p save_map_timeout:=60.0 -p use_sim_time:=true \
    > "$LOG/map_save.log" 2>&1
  if [ -f "$LOG/slam_map/slam_map.pgm" ]; then
    echo "[$R] ✔ 地图已保存: $LOG/slam_map/slam_map.pgm"
    python3 - "$LOG/slam_map/slam_map.pgm" <<'PYEOF'
import sys
from collections import Counter
try:
    from PIL import Image
    img = Image.open(sys.argv[1]).convert('L')
    d = list(img.getdata()); c = Counter(d); tot = len(d)
    free = sum(n for v, n in c.items() if v >= 250)
    occ = sum(n for v, n in c.items() if v <= 80)
    print("    %dx%d  free %.1f%%  occupied %.1f%%  unknown %.1f%%"
          % (img.size[0], img.size[1], 100*free/tot, 100*occ/tot, 100*(tot-free-occ)/tot))
except Exception as e:
    print("    （PIL 统计失败：%s）" % e)
PYEOF
  else
    echo "[$R] !! 存图失败，见 map_save.log"
  fi
fi

# ── 清理（显式 PID + DDS 持有者，写法照抄 dwb_round.sh 的已验证版本）──
# ⚠️ 不要用 `pgrep -f <pat>` / `pkill -f` 做收尾：它会匹配到本脚本自身的命令行。
#    用 lsof 找 DDS 持有者并按 cmdline 排除 ros2 daemon。
echo "[$R] ===== 清理（显式 PID）====="
while read -r p; do
  [ -n "$p" ] && kill -TERM "$p" 2>/dev/null && echo "  TERM $p"
done < "$PIDF"
sleep 8
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
[ -n "${NAV_PARAMS_NS:-}" ] && rm -f "$NAV_PARAMS_NS" && echo "  已删除临时参数文件"
echo "[$R] 地图在 $LOG/slam_map/"
echo "[$R] 脚本结束"
