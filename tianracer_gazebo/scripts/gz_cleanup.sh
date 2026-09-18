#!/usr/bin/env bash
# 清理 tianracer gz-sim 残留进程与窗口。
# 约束：每次启动 gz-sim 前必须运行本脚本（tianracer_on_racetrack.launch.py 启动时也会自动执行），
# 避免残留 gz server / GUI 窗口 / ROS 节点累积导致环境脏、渲染与端口冲突。

set -u

echo "[gz_cleanup] 清理 tianracer gz-sim 残留进程..."

# gz-sim 服务端/客户端（进程名可能是 ign gazebo 或 gz sim，两个都要匹配）
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "ruby /usr/bin/ign gazebo" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gz_server" 2>/dev/null

# 本项目 launch 启动的 ROS 节点
pkill -9 -f "servo_commands" 2>/dev/null
pkill -9 -f "transform.py" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
pkill -9 -f "spawner" 2>/dev/null
pkill -9 -f "ros_gz_sim" 2>/dev/null

# 孤儿节点（world 或 install 路径残留）。注意：脚本自身 cmdline 含
# install/tianracer_gazebo，必须排除 $$，否则脚本会杀掉自己。
for pid in $(pgrep -f "install/tianracer_gazebo" 2>/dev/null); do
  [ "$pid" = "$$" ] && continue
  kill -9 "$pid" 2>/dev/null
done

sleep 1
remaining=$(ps -ef | grep -E "ign gazebo|ruby /usr/bin/ign gazebo|servo_commands|install/tianracer_gazebo" | grep -v grep | grep -v "$$" | wc -l)
echo "[gz_cleanup] 剩余残留进程: $remaining"
if [ "$remaining" -gt 0 ]; then
  echo "[gz_cleanup] 仍有残留，请手动检查:"
  ps -ef | grep -E "ign gazebo|servo_commands|install/tianracer_gazebo" | grep -v grep | head -5
fi
