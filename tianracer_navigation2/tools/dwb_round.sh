#!/usr/bin/env bash
# =============================================================================
# DWB 赛程测试启动器（一轮 = 起 gz+nav2+裁判 → /start → 等结束 → 清理）
# =============================================================================
# 用法：
#   bash tools/dwb_round.sh <轮次标签>              # 例：bash tools/dwb_round.sh sim12
#   OUT=/path/to/logs bash tools/dwb_round.sh sim12 # 换输出目录（默认 /tmp/dwbround/<标签>）
#   WS=/path/to/ws    bash tools/dwb_round.sh sim12 # 换工作区（默认取本仓库根）
# 配套：dwb_sampler.py（本脚本自动起）、dwb_analyze.py（出报告）
#   分析：python3 tools/dwb_analyze.py <轮次目录>
#
# 五条防护（都是本项目踩过多次的坑，改之前先读对应段的注释）：
#   1. 前置残留检查：上一轮进程未回收则拒绝开跑（不能用 pkill -f，会自匹配误杀自己）
#   2. /dev/shm 孤儿 FastRTPS 段清理：跨轮累积会让 Nav2 的 DDS 发现超时卡死
#   3. 就绪等待设墙钟 150s 上限，避免 DDS 卡死时空转十几分钟跑出废数据
#   4. 单实例锁（flock）：整个仿真栈是独占资源
#   5. 控制器激活门禁：spawner 在负载下会静默 FATAL，产出底盘不动的废轮
#
# 退出码：0 有效轮 / 2 前置文件检查失败 / 3 无效轮（reason 见 dwb_verdict.py）/
#         4 未跑完 / 5 拿不到单实例锁
# 注意：source ROS setup.bash 时不能开 set -u（AMENT_TRACE_SETUP_FILES 未绑定会中止）。
set -o pipefail
TOOLS_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS="${WS:-$(cd "$TOOLS_DIR/../.." && pwd)}"
R="${1:-r1}"
# PLANNER: 用哪套局部规划器参数（= params/<PLANNER>_nav2_params.yaml 的名字）。
# 默认 navfn_dwb（本脚本原名如此）；跑 MPPI 时：PLANNER=navfn_mppi bash tools/dwb_round.sh <标签>
# 注意：dwb_recorder.py 订阅 DWB 特有的 /evaluation，MPPI 不发布该话题，
#       故非 DWB 规划器时自动跳过记录器（eval.csv 会缺，state.csv 仍由 sampler 产出）。
PLANNER="${PLANNER:-navfn_dwb}"
LOG="${OUT:-/tmp/dwbround}/$R"
mkdir -p "$LOG"
PIDF=$LOG/pids.txt

# ── 单实例锁 ─────────────────────────────────────────────────────
# 仿真栈（gz+Nav2+裁判）是独占资源：并发会互抢 DDS 与 /dev/shm 段、把对方的仿真当成
# 「上一轮残留」而拒绝启动、用同样的轮次名互相覆写目录、并交错写同一份日志。
# 2026-09-16 实测过一次（一个守候因 kill 打偏成孤儿），10 次尝试里 6 次互相拒绝。
#
# 取锁放在所有重活之前，否则两套 gz/Nav2 会先各自起起来再互相拒绝。
# 拿不到即退出（不等待），退避策略交给调用方，见 dwb_round_retry.sh 的 rc=5。
# 锁随 fd 关闭自动释放；DWB_LOCK_HELD=1 供已持锁的包装脚本透传。
# ⚠️ 下面每个后台命令都必须显式 `9>&-` 关掉 fd9：子进程会继承打开的 fd，只要有一个
#    后台进程活过本脚本（被 SIGKILL/中断时就会），锁就被它一直持有，之后每一轮都 rc=5。
#    实测过：父进程退出后锁仍被 `(sleep 6) &` 持有；子进程 9>&- 后立刻可再取。
if [ "${DWB_LOCK_HELD:-0}" = "1" ]; then
  echo "[$R] 单实例锁：已由调用方持有（DWB_LOCK_HELD=1），跳过"
else
  LOCKFILE="${LOCKFILE:-$(dirname "$LOG")/.dwb_round.lock}"
  HOLDERFILE="$LOCKFILE.holder"
  # ⚠️ 必须用 >>：> 会截断锁文件，竞争者一启动就把持有者的诊断信息清空（实测过）。
  # 故诊断信息单独放 $HOLDERFILE，锁文件本身只作锁用、内容无意义。
  exec 9>>"$LOCKFILE"
  if ! flock -n 9; then
    holder=$(cat "$HOLDERFILE" 2>/dev/null)
    echo "[$R] ★另一个 dwb_round.sh 正在运行（${holder:-持有者信息不可读}；锁 $LOCKFILE）"
    echo "[$R]   整个仿真栈是独占资源，并发会互抢 DDS/轮次目录 ⇒ 本轮直接退出（rc=5）"
    exit 5
  fi
  printf 'pid=%s round=%s since=%s\n' "$$" "$R" "$(date '+%F %T')" > "$HOLDERFILE"
  echo "[$R] 已取得单实例锁（$LOCKFILE，持有者 PID $$）"
fi

# 取锁之后才动轮次目录里的文件：否则被拒的那一次会先清空持锁轮的 pids.txt、
# 删掉它写的门禁标记 —— 正是这把锁要防的那种互相覆写。
: > "$PIDF"
# 清掉上一轮同名运行留下的门禁标记，否则一次失败的轮次会让后续同名重跑即使控制器
# 正常也被判 INVALID。只删这一个标记：轮次目录里的日志是现场证据，不能碰。
rm -f "$LOG/controllers_dead"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ── 文件存在性检查：源码树 + install **双侧** ────────────────────
# ⚠️ 必须查两侧。launch 经 get_package_share_directory 读的是 install/，
# 而本工作区是 --symlink-install —— 它只为【构建时已存在】的文件建软链，
# **新建**的 params/launch/world/检查门 文件不会自动出现在 install/。
# 只查源码树会让脚本顺利通过、launch 却报
#   [ERROR] [launch]: No such file or directory: .../install/<包>/share/<包>/...
# 然后每轮白等满 150s 就绪超时并被误判成「DDS 响应丢失卡死」。
# 2026-09-15 实测为此浪费 15 次 × 150s ≈ 37 分钟。
# 修法：colcon build --symlink-install --packages-select <包>
check_pkg_file() {   # $1=包名  $2=包内相对路径  $3=用途说明
  local pkg="$1" rel="$2" what="$3"
  local src="$WS/$pkg/$rel"
  local inst="$WS/install/$pkg/share/$pkg/$rel"
  if [ ! -f "$src" ]; then
    echo "[$R] !! 找不到 $what（源码树）: $pkg/$rel"
    exit 2
  fi
  # -f 会跟随软链：软链断了也落进这一支
  if [ ! -f "$inst" ]; then
    echo "[$R] !! 找不到 $what（install）: install/$pkg/share/$pkg/$rel"
    echo "[$R]    源码树里有、install 里没有或软链已断 ⇒ 多半是新增文件后没重建包。"
    echo "[$R]    修法: colcon build --symlink-install --packages-select $pkg"
    exit 2
  fi
}

check_pkg_file tianracer_navigation2 "params/${PLANNER}_nav2_params.yaml" "局部规划器参数"
echo "[$R] 局部规划器参数: ${PLANNER}"

# ── 赛道选择 ─────────────────────────────────────────────────────
# TIANRACER_WORLD 同时驱动三处（与 judge_system.py:112 / position_check.py 的约定一致）：
#   · world 文件      worlds/${WORLD}.world
#   · 检查门          waypoint_race/${WORLD}_check_points.yaml
#   · 地图            maps/${WORLD}.yaml（nav2.launch.py 的 map:= 参数）
# 仓库内带完整三件套的赛道：tianracer_racetrack(默认) / racetrack_1 / raicom / test_indoor
WORLD="${WORLD:-tianracer_racetrack}"
export TIANRACER_WORLD="$WORLD"
check_pkg_file tianracer_gazebo "worlds/${WORLD}.world" "world 文件"
check_pkg_file tianracer_gazebo "waypoint_race/${WORLD}_check_points.yaml" "检查门文件"
# 地图只查源码树：它是以**绝对路径**经 nav2.launch.py 的 map:= 传入的，
# 不经 get_package_share_directory，故 install 侧没有也不影响（新建地图无需重建包）。
MAPFILE="$WS/tianracer_gazebo/maps/${WORLD}.yaml"
if [ ! -f "$MAPFILE" ]; then
  echo "[$R] !! 找不到地图: maps/${WORLD}.yaml"; exit 2
fi
echo "[$R] 赛道: ${WORLD}（world + 检查门 + 地图 三件套已核对；world/检查门已核对 install 侧）"

# ── 前置残留检查 ─────────────────────────────────────────────────
echo "[$R] ===== 前置残留检查 ====="
resid=$(ps -eo pid,cmd --no-headers | awk '
  /snapshot-bash|dwb_round\.sh|ros2cli\.daemon|ros2-daemon|awk |ps -eo/ {next}
  /ign gazebo|ros2 launch|nav2_[a-z]|judge_system|judge_display|servo_commands|transform\.py|dwb_sampler|dwb_recorder|door_markers|robot_state_publisher|parameter_bridge/ {print "  " $0}
')
if [ -n "$resid" ]; then
  echo "[$R] 检测到上一轮残留进程，拒绝开始本轮："
  echo "$resid"
  echo "[$R] 请先按显式 PID 清理后重跑"
  exit 2
fi
echo "[$R] 无残留，继续"

# ── 重置 ros2 daemon ─────────────────────────────────────────────
# 必须做，且必须在下面的孤儿段清理【之前】。陈旧 daemon 会让 Nav2 bringup 卡死
# （2026-09-15 一天内撞了 3 次），表现是：
#   · 脚本侧：`Nav2 150s 内未就绪（managed=0/1 lifecycle=unconfigured）`
#   · nav2.log：global_costmap "Timed out waiting for transform from base_link to
#     map ... frame does not exist"，controller_server configure 也超时
# 此时仿真栈本身没问题（gz 起得来、/clock 正常），纯粹是发现层卡住。
# 停掉后 daemon 会在下一次 ros2 调用时自动重启，无需手动拉起；
# 它同时会释放自己持有的 /dev/shm/fastrtps_* 段，使下面的清理更彻底。
echo "[$R] ===== 重置 ros2 daemon ====="
if timeout 30 ros2 daemon stop >/dev/null 2>&1; then
  echo "[$R] daemon 已停（下次 ros2 调用时自动重启）"
else
  echo "[$R] daemon 停止未成功（可能本就未运行），继续"
fi
sleep 2
echo "[$R] 停后 DDS 持有者: $(lsof /dev/shm/fastrtps_* 2>/dev/null | awk 'NR>1{print $2}' | sort -u | tr '\n' ' ')"

# ── 清理孤儿 FastRTPS 段与信号量（只删 lsof 查不到持有者的）────────
# ⚠️ 必须同时覆盖两族，**不能用 `fastrtps_*` 一个 glob 了事**：
#   · 共享内存段   /dev/shm/fastrtps_portNNNN
#   · 端口互斥信号量 /dev/shm/sem.fastrtps_portNNNN_mutex  ← 点号开头，旧 glob 匹配不到
# 2026-09-17 实测：只清段、不清信号量，累积了 19 个无持有者的 sem.fastrtps_*_mutex。
# 机制（**推断，未闭环**）：新参与者创建共享内存端口时若撞上「名字相同、持有者已死」
# 的信号量会阻塞在创建阶段——正好打在每个新 DDS 参与者上，而**最先创建的两个
# 节点（map_server / amcl）最脆弱**（实测失败签名：它们连 lifecycle 横幅都不打印，
# 后启动的 controller_server/planner_server 全部正常）。故此处按族逐个删除。
free_shm_dds() {
  local sf h
  for sf in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
    [ -e "$sf" ] || continue
    h=$(lsof "$sf" 2>/dev/null | awk 'NR>1{print $2}' | sort -u)
    [ -z "$h" ] || continue      # 被持有的（含运行中的 daemon）一律保留
    rm -f "$sf" 2>/dev/null
  done
}
shm_before=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
free_shm_dds
shm_after=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
echo "[$R] /dev/shm 孤儿清理（段+信号量）: $shm_before -> $shm_after 个（被持有的一律保留）"

# ── 启动 gz + control ────────────────────────────────────────────
echo "[$R] ===== 启动 gz + control ====="
nohup ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py gui:=false world:="${WORLD}.world" \
  > "$LOG/gz.log" 2>&1 9>&- &
echo $! >> "$PIDF"

# ⚠️ 就绪探针一律加 --no-daemon（2026-09-17 修正）。
# 此前注释声称「改用 topic 就绕开了 daemon」——**那是错的**：`ros2 topic echo` 走
# ros2topic/verb/echo.py 的 NodeStrategy，照样依赖 daemon。实证：daemon 卡死（监听
# 127.0.0.1:11511 却从不 accept，Recv-Q 堆积）时，下面这个探针每次烧满 5s 超时 +
# 2s sleep = 7s × 60 次 ≈ **7 分钟**空转，而 gz 早就绪，导航因此从未启动。
# --no-daemon 让 CLI 直接建轻量节点（实测可用），彻底切断这条依赖。
for i in $(seq 1 60); do
  if timeout 5 ros2 topic echo --no-daemon --once /clock >/dev/null 2>&1; then echo "[$R] /clock OK"; break; fi
  sleep 2
done

# ── 启动 nav2（DWB）──────────────────────────────────────────────
# world:=$WORLD 决定 AMCL 初始位姿（nav2.launch.py 从 track_spawn 取该赛道出生位姿
# 改写 amcl.initial_pose）。不传则回退默认赛道，与 gz 里车的位置不符 → 定位发散。
echo "[$R] ===== 启动 nav2 ($PLANNER) ====="
nohup ros2 launch tianracer_navigation2 nav2.launch.py use_sim_time:=true use_rviz:=false use_planner:=$PLANNER \
  map:="$MAPFILE" world:="$WORLD" \
  > "$LOG/nav2.log" 2>&1 9>&- &
echo $! >> "$PIDF"

ok=0
deadline=$(( $(date +%s) + 150 ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  n=$(grep -c "Managed nodes are active" "$LOG/nav2.log" 2>/dev/null); n=${n:-0}
  lc=$(timeout 5 ros2 lifecycle get --no-daemon /bt_navigator 2>/dev/null | head -1)
  pubs=$(timeout 5 ros2 topic info --no-daemon /navigate_to_pose/_action/status 2>/dev/null | grep -i "Publisher count" | grep -o "[0-9]\+" | head -1); pubs=${pubs:-0}
  if [ "$n" -ge 2 ] && echo "$lc" | grep -q active && [ "$pubs" -ge 1 ]; then
    echo "[$R] Nav2 就绪: managed=$n lifecycle=$lc pubs=$pubs"; ok=1; break
  fi
  sleep 3
done
if [ "$ok" != 1 ]; then
  echo "[$R] Nav2 150s 内未就绪（managed=$n lifecycle=$lc pubs=$pubs）→ 判定 DDS 响应丢失卡死，跳过本轮"
fi

# ── 控制器激活门禁 ───────────────────────────────────────────────
# tianracer_control.launch.py 用**单个 spawner + --activate-as-group** 加载三个控制器，
# 而 spawner 的单次服务调用有 10s 内部超时（--controller-manager-timeout 只管等 CM
# 服务出现，不管单次调用）。负载高时 load_controller 响应超时 → spawner 重试撞上
# 「already loaded」→ FATAL 退出，车轮/转向控制器**从未激活**（现场见 ver1b 的 gz.log）。
# 后果隐蔽：车一步不动、odom 恒 0，但 Nav2 侧完全正常（cmd_vx 照发），
# 裁判 10s 后判 stopped、move_dis 0.00m —— 整轮白跑，且旧判定还会算它 VALID。
#
# 只查两个**运动相关**控制器：joint_state_broadcaster 只喂 /joint_states，缺它不影响
# 车轮转动，计入会引入假 INVALID（仅报告其状态供诊断）。
# 只在 Nav2 已就绪时做 —— 未就绪本来就会整段跳过赛程，不必再花 10~30s 查控制器。
if [ "$ok" = 1 ]; then
  echo "[$R] ===== 控制器激活门禁 ====="
  CTRL_OK=0
  QUERIED=0     # 是否真的拿到过查询结果（区别于「查了、确实没激活」）
  for attempt in 1 2 3; do
    # ⚠️ 必须关色 + 再剥一层 ANSI：ros2 CLI 默认彩色，active 实际是 \e[92mactive\e[0m，
    # 而 `grep -w` 要求词边界、`[92m` 的结尾是词字符 `m` ⇒ 匹配失败。
    # 2026-09-16 这个假阳性曾把整批 test_indoor 轮次误判作废，而三个控制器其实全 active。
    # timeout 取 20s：重负载下 CM 的响应会明显变慢，10s 太容易假超时。
    clist=$(RCUTILS_COLORIZED_OUTPUT=0 timeout 20 ros2 control list_controllers -c controller_manager 2>/dev/null \
            | sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g')
    if [ -z "$clist" ]; then
      # 查询本身失败 ≠ 控制器没激活。若照旧判「未激活」，就会对不存在的控制器反复
      # load_controller（每次 30s 超时）最后写假标记，把一轮本无问题的轮次判成
      # controllers_inactive —— 既误判又白等约 3 分钟。故空输出只重试，不下结论。
      echo "[$R]   尝试 $attempt: list_controllers 无输出（CM 未响应），重试查询"
      sleep 3
      continue
    fi
    QUERIED=1
    missing=""
    for c in wheel_velocity_controller steering_position_controller; do
      # -w 排除 "inactive"（'active' 前是词字符 n，无词边界）
      if ! echo "$clist" | grep -F -- "$c" | grep -qw active; then missing="$missing $c"; fi
    done
    jsb=$(echo "$clist" | grep -F -- joint_state_broadcaster | tr -s ' \t' ' ')
    echo "[$R]   尝试 $attempt: 未激活:${missing:- 无} | jsb: ${jsb:-（未列出）}"
    [ -z "$missing" ] && { CTRL_OK=1; break; }
    for c in $missing; do
      if timeout 30 ros2 control load_controller --set-state active "$c" -c controller_manager >/dev/null 2>&1; then
        echo "[$R]     $c → 激活成功"
      else
        echo "[$R]     $c → 激活失败"
      fi
    done
    sleep 3
  done

  if [ "$CTRL_OK" != 1 ] && [ "$QUERIED" = 1 ]; then
    # 标记文件让 dwb_verdict.py 判 INVALID/controllers_inactive（rc=3，可重跑）；
    # 同时把 ok 置 0 跳过整个赛程段，不必再白等 480s 与起裁判。
    echo "[$R] ★控制器确认未激活（底盘不会动）→ 跳过赛程段，标记本轮 INVALID 待重跑"
    : > "$LOG/controllers_dead"
    ok=0
  elif [ "$CTRL_OK" != 1 ]; then
    # 始终查不到状态 → 不下结论、不写标记，本轮照常跑。真死锁的轮次由 verdict 的
    # no_motion 判据兜住（车不动必 points=0 且 move_dis=0）。
    echo "[$R] ⚠ 始终查不到控制器状态（CM 未响应），不写门禁标记，本轮继续"
  fi
fi

if [ "$ok" = 1 ]; then
echo "[$R] ===== 参数实际加载值（确认改动生效）====="
{
  # 按规划器选关键键：确认配置真的加载了（不同规划器的参数面不同）
  case "$PLANNER" in
    navfn_dwb*) KEYS="sim_time min_vel_x max_vel_x max_vel_theta" ;;
    navfn_teb)  KEYS="max_vel_x max_vel_theta min_obstacle_dist weight_obstacle" ;;
    navfn_mppi*)
      KEYS="vx_max vx_min wz_max time_steps batch_size model_dt temperature motion_model" ;;
    *)          KEYS="max_vel_x max_vel_theta" ;;
  esac
  for k in $KEYS; do
    printf 'FollowPath.%-20s = ' "$k"
    timeout 5 ros2 param get /controller_server "FollowPath.$k" 2>/dev/null | tail -1
  done
  printf 'smoother max_velocity%-15s = ' ''
  timeout 5 ros2 param get /velocity_smoother max_velocity 2>/dev/null | tail -1
} | tee "$LOG/params_loaded.txt"

echo "[$R] ===== 启动裁判 ====="
nohup ros2 launch tianracer_gazebo judge.launch.py > "$LOG/judge.log" 2>&1 9>&- &
echo $! >> "$PIDF"
sleep 6

echo "[$R] ===== 先起采样器（早于 /start）====="
nohup python3 "$TOOLS_DIR/dwb_sampler.py" "$LOG/samples.csv" > "$LOG/sampler.log" 2>&1 9>&- &
echo $! >> "$PIDF"

# 记录器：三层记录（状态层 state.csv / 决策层 eval.csv / 事件层 events/*.json）。
# 决策层来自 DWB 的 /evaluation（一条约 798KB × 18Hz × 400 候选），故只在减速时存现场。
# ⚠️ 只有 DWB 发布 /evaluation，MPPI 不发布（可视化走 TrajectoryVisualizer），
#    非 DWB 时必须跳过，否则 recorder 空转且白占 CPU 影响成绩。
#
# 判定不能写死成某个名字：本仓库命名是 <全局规划器>_<局部规划器>，DWB 有四种拼法
# （navfn_dwb / navfn_dwb_r1 / smac_dwb / theta_star_dwb）。此处原为精确相等
# `[ "$PLANNER" = "navfn_dwb" ]`，导致除 navfn_dwb 外三种全部走 else，
# racetrack_1 的候选级数据被静默跳过。故同时覆盖 navfn_dwb* 与 *_dwb 两种拼法。
case "$PLANNER" in
  navfn_dwb*|*_dwb)
    nohup python3 "$TOOLS_DIR/dwb_recorder.py" "$LOG/rec" > "$LOG/recorder.log" 2>&1 9>&- &
    echo $! >> "$PIDF"
    ;;
  *)
    echo "[$R] 跳过 dwb_recorder（$PLANNER 不是 DWB 变体，不发布 /evaluation）"
    echo "skipped: $PLANNER 不发布 /evaluation" > "$LOG/recorder.log"
    ;;
esac
sleep 5

echo "[$R] ===== /start @ $(date +%s.%N) ====="
# 用常驻 rclpy client 发（judge_start.py），不用 `ros2 service call`：
# 后者每次新起一个 Python 进程（实测最轻的 ros2 CLI 命令也要 0.30~0.36s，
# 服务调用还要加发现与往返），这段开销会被算进「起步死时间」。真实用户点的是
# judge_display.py 的常驻 client（call_async），没有这部分，故此处对齐真实口径。
# t_start.txt 由 judge_start.py 在「服务就绪、请求发出的那一刻」写入。
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
for pat in "Controller patience exceeded" "failed to create plan" "No valid trajectories" "clear entirely the global_costmap" "Trajectory Hits Obstacle" "possible oscillation"; do
  c=$(grep -c "$pat" "$LOG/nav2.log" 2>/dev/null); c=${c:-0}
  printf '  %-34s %s\n' "$pat" "$c"
done

echo "[$R] ===== 采样计数 ====="
tail -2 "$LOG/sampler.log"

fi   # ← 结束 "if [ $ok = 1 ]" 赛程段（未就绪时整段跳过）

# ── 清理（显式 PID）─────────────────────────────────────────────
echo "[$R] ===== 清理（显式 PID）====="
while read -r p; do
  [ -n "$p" ] && kill -TERM "$p" 2>/dev/null && echo "  TERM $p"
done < "$PIDF"
sleep 8
# 复查：仍持有 DDS 共享内存的 PID 逐个 KILL；按 cmdline 动态排除 ros2 daemon
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

# ── 轮次有效性判定 ───────────────────────────────────────────────
# 判据全部在 tools/dwb_verdict.py（含每条判据的现场依据与回归验证），此处只透传结论。
echo "[$R] ===== 轮次有效性判定 ====="
python3 "$TOOLS_DIR/dwb_verdict.py" "$LOG"
vrc=$?
# 原因不写死在这里，避免新增判据后文案与实际不符
reason=$(sed -n 's/.*reason=\([a-z_]*\).*/\1/p' "$LOG/verdict.txt" 2>/dev/null)
case "$vrc" in
  0) echo "[$R] 结论：有效轮，可计入统计" ;;
  3) echo "[$R] 结论：★本轮无效（reason=$reason）→ 不计入统计，需重跑" ;;
  4) echo "[$R] 结论：★本轮未跑完（reason=$reason）→ 不计入统计，需重跑" ;;
  *) echo "[$R] 结论：★dwb_verdict.py 异常退出（rc=$vrc），结果不可信，需重跑" ;;
esac
echo "[$R] 脚本结束"
exit $vrc
