#!/usr/bin/env bash
# =============================================================================
# 手动建图：起 gz + slam_toolbox（**不启导航栈**），然后用键盘遥控开
# =============================================================================
# 为什么手动建图优于自动巡游（实测结论）：
#   自动巡游 (mapping_round.sh + mapping_drive.py) 在本项目受阻于**建图死锁**：
#     slam_toolbox 的 minimum_travel_distance(0.5m) 要求车移动才处理扫描
#     → 车不动则 /map 恒为 0x0
#     → global_costmap 判 "Received map message is malformed. Rejecting."
#     → 全局规划永久失败 → 车不走 …… 闭环。
#   加暖机（先手发 /cmd_vel 走一段）能打破死锁（/map 由 0x0 变为有值），
#   但自动巡游的目标点常落在地图边界外（worldToMap failed），
#   末端的 90° 弯与窄通道自动规划不易通过。**人操作一扭就过去**，故改手动。
#
# 用法（两步，都必须在你自己的终端里跑）：
#   ① 起环境（本脚本）：
#        WORLD=test_indoor bash tianracer_navigation2/tools/mapping_manual_start.sh start
#   ② 另开一个终端遥控（需先 source）：
#        source /opt/ros/humble/setup.bash && source <工作区路径>/install/setup.bash
#        ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   ③ 走完赛道后在**第一个终端**回车，或另开终端跑：
#        WORLD=test_indoor bash tianracer_navigation2/tools/mapping_manual_start.sh save
#   ④ 清理：
#        bash tianracer_navigation2/tools/mapping_manual_start.sh stop
# -----------------------------------------------------------------------------
# 注意：**不要**同时跑导航栈 —— controller_server 会与键盘抢 /cmd_vel。
set -o pipefail
TOOLS_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS="${WS:-$(cd "$TOOLS_DIR/../.." && pwd)}"
ACTION="${1:-start}"
WORLD="${WORLD:-test_indoor}"
export TIANRACER_WORLD="$WORLD"
LOG="${OUT:-/tmp/dwbmapping}/manual_${WORLD}"
mkdir -p "$LOG"
PIDF=$LOG/pids.txt

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

case "$ACTION" in
start)
  : > "$PIDF"
  echo "[manual] 赛道: $WORLD  (日志: $LOG)"

  # 前置残留检查（排除自身与 awk，避免自匹配误报）
  resid=$(ps -eo pid,cmd --no-headers | awk '
    /snapshot-bash|mapping_manual|ros2cli\.daemon|claude|opencode|awk |grep |teleop_twist/ {next}
    /ign gazebo|ros2 launch|nav2_[a-z]|slam_toolbox|servo_commands|transform\.py|robot_state_publisher|parameter_bridge|amcl|map_server/ {print "  " $0}' | head -10)
  if [ -n "$resid" ]; then
    echo "[manual] !! 检测到残留进程，拒绝开跑："; echo "$resid"; exit 2
  fi
  echo "[manual] 无残留"

  # /dev/shm 孤儿段清理（只删无进程持有的）
  held_shm=$(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $NF}' | sort -u)
  for sf in $(ls /dev/shm/fastrtps_* 2>/dev/null); do
    case "$held_shm" in *"$sf"*) ;; *) rm -f "$sf" 2>/dev/null ;; esac
  done

  echo "[manual] 启动 gz + control（gui:=false，无 3D 界面）..."
  nohup ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py gui:=false world:="${WORLD}.world" \
    > "$LOG/gz.log" 2>&1 &
  echo $! >> "$PIDF"
  for i in $(seq 1 60); do
    if timeout 5 ros2 topic echo --once /clock >/dev/null 2>&1; then echo "[manual] /clock OK"; break; fi
    sleep 2
  done

  # slam_toolbox（只有它；不启导航栈）
  SLAM_PARAMS="$WS/install/tianracer_slam/share/tianracer_slam/param/mapper_params_online_async.yaml"
  echo "[manual] 启动 slam_toolbox ..."
  nohup ros2 launch slam_toolbox online_sync_launch.py \
    use_sim_time:=true autostart:=true \
    slam_params_file:="$SLAM_PARAMS" \
    > "$LOG/slam.log" 2>&1 &
  echo $! >> "$PIDF"

  for i in $(seq 1 40); do
    st=$(timeout 5 ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1)
    if echo "$st" | grep -q active; then echo "[manual] slam_toolbox: $st"; break; fi
    sleep 3
  done

  echo
  echo "[manual] ===================================================="
  echo "[manual] 环境已就绪。请**另开一个终端**遥控："
  echo "[manual]   source /opt/ros/humble/setup.bash"
  echo "[manual]   source $WS/install/setup.bash"
  echo "[manual]   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
  echo "[manual]"
  echo "[manual] 走完赛道后，在本目录跑："
  echo "[manual]   WORLD=$WORLD bash $TOOLS_DIR/mapping_manual_start.sh save"
  echo "[manual] ===================================================="
  ;;

save)
  echo "[manual] 存图到 $LOG/slam_map/ ..."
  mkdir -p "$LOG/slam_map"
  timeout 120 ros2 run nav2_map_server map_saver_cli -f "$LOG/slam_map/slam_map" \
    --ros-args -p save_map_timeout:=60.0 -p use_sim_time:=true \
    > "$LOG/map_save.log" 2>&1
  if [ -f "$LOG/slam_map/slam_map.pgm" ]; then
    echo "[manual] ✔ 地图已保存: $LOG/slam_map/slam_map.pgm"
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
    print("    （统计失败：%s）" % e)
PYEOF
  else
    echo "[manual] !! 存图失败，见 $LOG/map_save.log"
    tail -5 "$LOG/map_save.log"
  fi
  ;;

stop)
  echo "[manual] 清理..."
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
  fi
  echo "[manual] 清理后剩余 DDS 持有者: $(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $2}' | sort -u | tr '\n' ' ')"
  ;;

*)
  echo "用法: WORLD=<赛道> bash $0 {start|save|stop}"
  exit 2
  ;;
esac
