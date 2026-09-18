#!/usr/bin/env bash
# 启动 tianracer gz-sim 仿真（带前置清理）。
# 约束：每次启动前先清理上次残留的 gz-sim 进程/窗口，避免累积。
# gz 服务端由本脚本直接启动（launch 内启动偶发挂起），再以 start_gz:=false 启动 ROS 栈。
# 用法：bash run_sim.sh [gui:=true|false] [world:=<world>.world] [额外 launch 参数...]

set -eu

T_ALL0=$(date +%s)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SCRIPT_DIR="$SCRIPT_DIR"
# 兼容已安装（share）与源码（scripts）两种路径
[ -f "$PKG_SCRIPT_DIR/gz_cleanup.sh" ] || PKG_SCRIPT_DIR="$(dirname "$SCRIPT_DIR")/scripts"
[ -f "$PKG_SCRIPT_DIR/gz_cleanup.sh" ] || PKG_SCRIPT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")/share/tianracer_gazebo/scripts"

echo "[run_sim] 清理上次残留的 gz-sim 进程/窗口..."
bash "$PKG_SCRIPT_DIR/gz_cleanup.sh"

# 解析 world/gui 参数
GUI="false"
WORLD="tianracer_racetrack.world"
EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    gui:=*) GUI="${arg#gui:=}" ;;
    world:=*) WORLD="${arg#world:=}" ;;
    *) EXTRA_ARGS+=("$arg") ;;
  esac
done

PKG_SHARE="$(ros2 pkg prefix tianracer_gazebo)/share/tianracer_gazebo"
WORLD_PATH="$PKG_SHARE/worlds/$WORLD"

echo "[run_sim] 直接启动 gz 服务端: ign gazebo --force-version 6 -r [-s] $WORLD_PATH"
# 关键：设置 gz 系统插件路径，否则 gz server 找不到 libgz_ros2_control-system.so 与 odometry 插件。
# 优先从环境动态推导（不依赖发行版路径写死）：
#   - gz_ros2_control 的 lib 目录（libgz_ros2_control-system.so 所在）
#   - ignition-gazebo6 系统插件目录（libignition-gazebo-*-system.so 所在）
ROS_LIB_DIR="$(ros2 pkg prefix gz_ros2_control 2>/dev/null)/lib"
# ignition-gazebo6 的 .pc 无 plugindir 变量，用 libdir 拼接；失败回退发行版默认路径
GZ_PLUGIN_DIR="$(pkg-config --variable=libdir ignition-gazebo6 2>/dev/null)/ign-gazebo-6/plugins"
[ -d "$GZ_PLUGIN_DIR" ] || GZ_PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROS_LIB_DIR:$GZ_PLUGIN_DIR"
# 设置模型/资源路径：世界文件里的 mesh（tianracer_racetrack/meshes/*.dae 等）是模型相对路径，
# 需要 GZ_SIM_RESOURCE_PATH 指向 worlds 目录才能找到网格与纹理，否则赛道渲染为黑色无纹理
export GZ_SIM_RESOURCE_PATH="$PKG_SHARE/worlds"
export IGN_GAZEBO_RESOURCE_PATH="$PKG_SHARE/worlds"
GZ_ARGS=(ign gazebo --force-version 6 -r)
if [ "$GUI" = "false" ]; then
  GZ_ARGS+=(-s)
fi
GZ_ARGS+=("$WORLD_PATH")
nohup "${GZ_ARGS[@]}" > /tmp/tianracer_gz_server.log 2>&1 &
GZ_PID=$!
echo "[run_sim] gz 服务端 PID=$GZ_PID (日志: /tmp/tianracer_gz_server.log)"

# 与 gz 启动并行预生成 URDF（省掉 launch 内 xacro 的串行等待，实测约 0.49s）。
# prebuilt_urdf 只有在预生成成功时才置 true，失败则回退到 launch 内生成。
NS="${TIANBOT_NAME:-}"
[ "$NS" = "/" ] && NS=""
PREBUILT_URDF="false"
xacro "$PKG_SHARE/urdf/tianracer_run.xacro" "prefix:='$NS'" > /tmp/tianracer_robot.urdf 2>/dev/null &
XACRO_PID=$!

# 等待 gz 服务端就绪（create 服务可用）
echo "[run_sim] 等待 gz 服务端就绪..."
T_WAIT0=$(date +%s)
for i in $(seq 1 30); do
  # 注意：gz CLI 单次调用约 2.4s（传输握手固定成本），故循环内不再额外 sleep
  if gz service -l 2>/dev/null | grep -q "/world/default/create"; then
    echo "[run_sim] gz 服务端已就绪（等待 $(( $(date +%s) - T_WAIT0 ))s）"
    break
  fi
  if [ "$i" = "30" ]; then
    echo "[run_sim] 警告: gz 服务端未就绪，继续尝试"
  fi
done

if wait "$XACRO_PID" 2>/dev/null; then
  PREBUILT_URDF="true"
else
  echo "[run_sim] 警告: URDF 预生成失败，回退到 launch 内生成"
fi

echo "[run_sim] 启动 ROS 栈（start_gz:=false）... [至此累计 $(( $(date +%s) - T_ALL0 ))s]"
ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py \
  "gui:=$GUI" "world:=$WORLD" "start_gz:=false" "prebuilt_urdf:=$PREBUILT_URDF" "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"
