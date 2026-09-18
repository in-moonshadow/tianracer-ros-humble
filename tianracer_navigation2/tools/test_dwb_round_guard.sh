#!/usr/bin/env bash
# =============================================================================
# dwb_round.sh「文件检查」回归测试
# =============================================================================
# 测什么：dwb_round.sh 开头的 check_pkg_file() 是否对 **源码树 + install 双侧** 都做检查。
#
# 为什么需要它（本测试守护的正是我自己踩过的坑）：
#   本工作区是 --symlink-install —— 它只为【构建时已存在】的文件建软链。**新建**的
#   params/launch/world/检查门 文件不会自动出现在 install/，而 launch 经
#   get_package_share_directory 读的是 install/。只查源码树会让脚本顺利通过、
#   launch 却报 No such file，然后每轮白等满 150s 就绪超时并被误判成
#   「DDS 响应丢失卡死」。2026-09-15 实测为此浪费 15 次 × 150s ≈ 37 分钟。
#   修法：colcon build --symlink-install --packages-select <包>
#
# 设计要点：第 2 段从 dwb_round.sh 里**抽出**真实的 check_pkg_file 函数体来测，
#   而不是另写一份等价逻辑 —— 另写一份只能证明我写对了，不能证明脚本里那段是对的。
#
# 用法：bash tools/test_dwb_round_guard.sh
# 退出码：0 = 全部通过；1 = 有失败项。可直接接进 CI 或提交前自检。
#
# 注意：测试**不会启动仿真**（用不存在的赛道 / 缺件赛道让脚本在文件检查处就退出）。
set -o pipefail
TOOLS_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS="${WS:-$(cd "$TOOLS_DIR/../.." && pwd)}"
fail=0

echo "=== 1) 语法检查 ==="
bash -n "$TOOLS_DIR/dwb_round.sh" && echo "  OK" || { echo "  FAIL"; exit 1; }

echo
echo "=== 2) 抽出真实函数，做四分支测试 ==="
# 从 dwb_round.sh 取 check_pkg_file 的定义（自 'check_pkg_file() {' 起至其收尾 '}'）
awk '/^check_pkg_file\(\) \{/{f=1} f{print} f&&/^\}$/{exit}' \
    "$TOOLS_DIR/dwb_round.sh" > /tmp/_guard_fn.$$.sh
echo "  抽出 $(wc -l < /tmp/_guard_fn.$$.sh) 行"
if grep -q 'install/\$pkg/share' "/tmp/_guard_fn.$$.sh"; then
  echo "  OK 函数含 install 侧检查"
else
  echo "  FAIL 抽出的函数不含 install 检查"; fail=1
fi

# 假工作区：造出「源码有 / install 无」这一关键场景
WS_REAL="$WS"                 # 保存真工作区（下面会把 WS 指向假工作区）
T=$(mktemp -d)
mkdir -p "$T/fakepkg/params"
: > "$T/fakepkg/params/only_in_src_params.yaml"
R=TEST
WS="$T"
# shellcheck disable=SC1090
source "/tmp/_guard_fn.$$.sh"

echo -n "  负例（源码有 / install 无，即本次要防的坑）: "
out=$( (check_pkg_file fakepkg "params/only_in_src_params.yaml" "测试参数") 2>&1 ); rc=$?
if [ "$rc" = 2 ] && echo "$out" | grep -q "测试参数（install）" \
   && echo "$out" | grep -q "colcon build --symlink-install --packages-select fakepkg"; then
  echo "OK（exit=2，且给出重建命令）"
else
  echo "FAIL (exit=$rc)"; echo "$out" | sed 's/^/      /'; fail=1
fi

echo -n "  负例（源码也没有）: "
out=$( (check_pkg_file fakepkg "params/nope.yaml" "测试参数") 2>&1 ); rc=$?
if [ "$rc" = 2 ] && echo "$out" | grep -q "（源码树）"; then echo "OK"; else echo "FAIL(exit=$rc)"; fail=1; fi

echo -n "  正例（两侧都有，install 侧为软链）: "
mkdir -p "$T/install/fakepkg/share/fakepkg/params"
ln -s "$T/fakepkg/params/only_in_src_params.yaml" \
      "$T/install/fakepkg/share/fakepkg/params/only_in_src_params.yaml"
out=$( (check_pkg_file fakepkg "params/only_in_src_params.yaml" "测试参数") 2>&1 ); rc=$?
if [ "$rc" = 0 ]; then echo "OK（无输出、不退出）"; else echo "FAIL(exit=$rc)"; echo "$out" | sed 's/^/      /'; fail=1; fi

echo -n "  负例（install 侧软链已断）: "
rm -f "$T/fakepkg/params/only_in_src_params.yaml"     # 删掉目标，制造断链
out=$( (check_pkg_file fakepkg "params/only_in_src_params.yaml" "测试参数") 2>&1 ); rc=$?
if [ "$rc" = 2 ]; then echo "OK（exit=2）"; else echo "FAIL(exit=$rc)"; fail=1; fi

rm -rf "$T" "/tmp/_guard_fn.$$.sh"
WS="$WS_REAL"                 # 必须恢复，否则下面会 cd 到已删除的临时目录（曾导致假通过）
[ -d "$WS/tianracer_navigation2" ] || { echo "  FAIL WS 恢复失败: $WS"; exit 1; }

echo
echo "=== 3) 真实配置：所有 规划器 × 就绪赛道 的源码与 install 两侧都应存在 ==="
planners=$(cd "$WS/tianracer_navigation2/params" && ls *_nav2_params.yaml 2>/dev/null | sed 's/_nav2_params.yaml$//')
for p in $planners; do
  s="$WS/tianracer_navigation2/params/${p}_nav2_params.yaml"
  i="$WS/install/tianracer_navigation2/share/tianracer_navigation2/params/${p}_nav2_params.yaml"
  if [ -f "$s" ] && [ -f "$i" ]; then printf '  OK   %-26s 两侧齐\n' "$p"
  else printf '  ⚠    %-26s 源码:%s install:%s\n' "$p" \
        "$([ -f "$s" ] && echo 有 || echo 无)" "$([ -f "$i" ] && echo 有 || echo 无)"; fail=1; fi
done
for w in $(cd "$WS/tianracer_gazebo/worlds" && ls *.world 2>/dev/null | sed 's/\.world$//'); do
  # 只检「竞速就绪」的赛道（源码侧三件套齐）。不完全的赛道（race_with_cones / room_mini
  # 缺检查门）在改动前也会 exit 2，不算回归，跳过。
  ready=1
  for rel in "worlds/${w}.world" "waypoint_race/${w}_check_points.yaml" "maps/${w}.yaml"; do
    [ -f "$WS/tianracer_gazebo/$rel" ] || ready=0
  done
  [ "$ready" = 0 ] && { printf '  --   %-26s 非竞速就绪（源码缺件），跳过\n' "$w"; continue; }
  ok=1
  for rel in "worlds/${w}.world" "waypoint_race/${w}_check_points.yaml"; do
    [ -f "$WS/install/tianracer_gazebo/share/tianracer_gazebo/$rel" ] || ok=0
  done
  if [ "$ok" = 1 ]; then printf '  OK   %-26s world+检查门 两侧齐\n' "$w"
  else printf '  ⚠    %-26s 源码齐但 install 缺 ⇒ 守卫会拦（正是它该做的）\n' "$w"; fail=1; fi
done

echo
echo "=== 4) 真实脚本集成测试（不启动仿真）==="
echo -n "  负例：赛道不存在 ⇒ 规划器检查通过、world 检查秒级拦住: "
out=$(cd "$WS" && PLANNER=navfn_dwb_r1 WORLD=__no_such_track__ timeout 30 \
      bash "$TOOLS_DIR/dwb_round.sh" guardtest 2>&1); rc=$?
if [ "$rc" = 2 ] && echo "$out" | grep -q "world 文件（源码树）" \
   && echo "$out" | grep -q "局部规划器参数: navfn_dwb_r1"; then
  echo "OK（exit 2，未启动仿真）"
else
  echo "FAIL(exit=$rc)"; echo "$out" | tail -6 | sed 's/^/      /'; fail=1
fi

# 正例不跑真实赛道——那会真的启动 gz，timeout 掐断后会留下残留进程。
# 改用 race_with_cones：world 两侧都在（⇒ 规划器检查与 world 检查都必须先通过），
# 但检查门缺失 ⇒ 在检查门那一关 exit 2。既证明前两项通过，又不启动仿真。
echo -n "  正例：world 两侧齐的赛道 ⇒ 前两项通过后停在检查门处: "
out=$(cd "$WS" && PLANNER=navfn_dwb_r1 WORLD=race_with_cones timeout 30 \
      bash "$TOOLS_DIR/dwb_round.sh" guardtest 2>&1); rc=$?
if [ "$rc" = 2 ] && echo "$out" | grep -q "检查门文件（源码树）" \
   && echo "$out" | grep -q "局部规划器参数: navfn_dwb_r1"; then
  echo "OK（exit 2，未启动仿真）"
else
  echo "FAIL(exit=$rc)"; echo "$out" | tail -6 | sed 's/^/      /'; fail=1
fi

# 收尾：确认没留下仿真残留。用 [x] 括号技巧避免 grep 自身命令行被匹配到
#（本项目有专门的记忆 [[proc-scan-self-match-trap]]，pkill/pgrep -f 的自匹配坑）。
resid=$(ps -eo cmd --no-headers | grep -E "[i]gn gazebo|[r]os2 launch" | head -3)
if [ -z "$resid" ]; then echo "  OK  未留下仿真残留"
else echo "  ⚠   有残留:"; echo "$resid" | sed 's/^/      /'; fail=1; fi

echo
if [ "$fail" = 0 ]; then echo "===== 全部通过 ====="; else echo "===== 有失败项 ====="; fi
exit $fail
