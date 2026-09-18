#!/usr/bin/env bash
# =============================================================================
# 地图配准采样轮：起 gz（**只起仿真，不起 Nav2/AMCL/裁判**）→ 传送取扫描 → 清理
# =============================================================================
# 为什么只起 gz：
#   · 配准要的位姿来自 /odom（gz 真值里程计），不需要 AMCL；
#   · 不需要导航，车由 /world/default/set_pose 传送，故不受窄道卡死率影响；
#   · 少一堆 DDS 参与者，启动快、失败面小。
#
# 用法：
#   WORLD=tianracer_racetrack bash tools/map_reg_round.sh <标签> [最大采样数]
#   产出：/tmp/mapreg/<标签>/scans.npz、poses.json
#   随后：python3 tools/map_reg_fit.py --scans /tmp/mapreg/<标签>/scans.npz \
#                                      --map tianracer_gazebo/maps/<赛道>.yaml
#
# 沿用 dwb_round.sh 的三条防护（本项目踩过多次，勿删）：前置残留检查（不能用
# pkill -f，会自匹配误杀自己）、/dev/shm 孤儿段清理、就绪等待用墙钟上限。
set -o pipefail
TOOLS_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS="${WS:-$(cd "$TOOLS_DIR/../.." && pwd)}"
R="${1:-reg1}"
MAXPOSES="${2:-60}"
LOG="${OUT:-/tmp/mapreg}/$R"
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
MAPFILE="$WS/tianracer_gazebo/maps/${WORLD}.yaml"
if [ ! -f "$MAPFILE" ]; then
  echo "[$R] !! 找不到地图: maps/${WORLD}.yaml"; exit 2
fi
echo "[$R] 赛道: ${WORLD}  采样上限 $MAXPOSES"

# ── 前置残留检查 ─────────────────────────────────────────────────
echo "[$R] ===== 前置残留检查 ====="
resid=$(ps -eo pid,cmd --no-headers | awk '
  /snapshot-bash|map_reg_round\.sh|ros2cli\.daemon|ros2-daemon|awk |ps -eo/ {next}
  /ign gazebo|ros2 launch|judge_system|servo_commands|transform\.py|robot_state_publisher|parameter_bridge/ {print "  " $0}
')
if [ -n "$resid" ]; then
  echo "[$R] 检测到残留进程，拒绝开始："; echo "$resid"
  echo "[$R] 请先按显式 PID 清理后重跑"; exit 2
fi
echo "[$R] 无残留，继续"

# ── 重置 ros2 daemon（陈旧 daemon 会让 DDS 发现卡死，见 dwb_round.sh 的说明）──
echo "[$R] ===== 重置 ros2 daemon ====="
timeout 30 ros2 daemon stop >/dev/null 2>&1 && echo "[$R] daemon 已停" || echo "[$R] daemon 本就未运行"
sleep 2

# ── 清理孤儿 FastRTPS 段（只删 lsof 查不到持有者的）──────────────
held_shm=$(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $NF}' | sort -u)
before=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
for sf in /dev/shm/fastrtps_*; do
  [ -e "$sf" ] || continue
  case "$held_shm" in *"$sf"*) ;; *) rm -f "$sf" 2>/dev/null ;; esac
done
echo "[$R] /dev/shm 孤儿段清理: $before -> $(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l) 个"

# ── 启动 gz（只仿真）─────────────────────────────────────────────
echo "[$R] ===== 启动 gz ====="
nohup ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py gui:=false \
  world:="${WORLD}.world" > "$LOG/gz.log" 2>&1 &
echo $! >> "$PIDF"

for i in $(seq 1 60); do
  timeout 5 ros2 topic echo --once /clock >/dev/null 2>&1 && { echo "[$R] /clock OK"; break; }
  sleep 2
done
# 传感器与 TF 就绪：无头环境软渲染下 lidar 首帧可能偏慢，给足 120s
ok=0
deadline=$(( $(date +%s) + 120 ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  if timeout 6 ros2 topic echo --once /scan >/dev/null 2>&1 \
     && timeout 6 ros2 topic echo --once /odom >/dev/null 2>&1; then ok=1; break; fi
  sleep 3
done
if [ "$ok" != 1 ]; then
  echo "[$R] !! /scan 或 /odom 120s 内未就绪，放弃本轮（见 $LOG/gz.log）"
else
  echo "[$R] /scan 与 /odom 就绪"

  # ── 采样 ───────────────────────────────────────────────────────
  echo "[$R] ===== 传送采样（最多 $MAXPOSES 个位姿）====="
  # grid/clearance 可覆盖：默认 0.6m 网格 + 0.35m 离墙，可在赛道自由区取满 60+ 点。
  # clearance 不宜大：0.98m 的窄道装不下 0.7m 的圆，给大了会把关键区域整个排除。
  timeout 900 python3 "$TOOLS_DIR/map_reg_scan.py" --map "$MAPFILE" --out "$LOG" \
    --max-poses "$MAXPOSES" --grid "${GRID:-0.6}" --clearance "${CLEAR:-0.35}" \
    2>&1 | tee "$LOG/scan.log"
  echo "[$R] 采样退出码: ${PIPESTATUS[0]}"
fi

# ── 清理（显式 PID）─────────────────────────────────────────────
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
echo "[$R] 脚本结束"
