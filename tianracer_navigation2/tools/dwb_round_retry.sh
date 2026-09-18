#!/usr/bin/env bash
# =============================================================================
# 带有效性判定的轮次包装器：无效轮自动重跑，直到拿到有效轮或耗尽尝试次数
# =============================================================================
# 用法：
#   bash tools/dwb_round_retry.sh <标签> [最大尝试次数]      # 默认 3
#   WORLD=test_indoor bash tools/dwb_round_retry.sh ti_x 5
#
# 设计取舍
#   · 每次尝试写【独立目录】<标签>_tryN，不覆盖上一次 —— 无效轮的现场（nav2.log /
#     judge.log）是判断「为什么没跑起来」的唯一证据，覆盖掉就没了。
#   · 判定交给 dwb_round.sh 末尾的 dwb_verdict.py，判据见该文件头。
#   · 只对「无效/未跑完」重试；正常的 0/9（跑出去后卡死）是**有效结果**，不重跑。
#   · 整个重试序列持单实例锁，别的 harness 会直接 rc=5 退出（见 dwb_round.sh 锁段）。
#
# 退出码：0 = 取得有效轮；1 = MAX 次均未取得有效轮；5 = 拿不到单实例锁
set -o pipefail
TOOLS_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
LABEL="${1:?用法: dwb_round_retry.sh <标签> [最大尝试次数]}"
MAX="${2:-3}"

# ── 单实例锁：在整个重试序列期间持有（2026-09-16 新增）──────────────
# 粒度取「整个序列」而不是「单轮」：一次 A/B 的所有重试属于同一组测量，
# 中间被另一个 harness 插进来会换掉机器负载状态，组内轮次就不可比了。
# 详见 dwb_round.sh 里「单实例锁」段的背景说明。
OUT_DIR="${OUT:-/tmp/dwbround}"
mkdir -p "$OUT_DIR"
LOCKFILE="${LOCKFILE:-$OUT_DIR/.dwb_round.lock}"
HOLDERFILE="$LOCKFILE.holder"
# 用 >> 而非 >：> 会截断锁文件，清掉持有者写下的诊断信息（见 dwb_round.sh 同段注释）。
exec 9>>"$LOCKFILE"
if ! flock -n 9; then
  holder=$(cat "$HOLDERFILE" 2>/dev/null)
  echo "########## $LABEL 无法取得单实例锁：另一个 harness 正在运行（${holder:-持有者信息不可读}）##########"
  echo "########## 锁文件 $LOCKFILE —— 等它跑完再重试，避免两套仿真互抢 ##########"
  exit 5
fi
printf 'pid=%s round=%s since=%s\n' "$$" "$LABEL(retry)" "$(date '+%F %T')" > "$HOLDERFILE"
echo "########## $LABEL 已取得单实例锁（PID $$）##########"

for i in $(seq 1 "$MAX"); do
  echo "########## $LABEL 第 $i/$MAX 次尝试 → ${LABEL}_try$i ##########"
  # DWB_LOCK_HELD=1：锁已由本脚本持有，子进程跳过取锁，否则会自我死锁。
  # 9>&-：别把锁的 fd 交给子进程 —— 它会继承给 gz/Nav2 等后台进程，
  # 那些进程活过本轮就会一直占着锁（见 dwb_round.sh 锁段实测）。
  OUT="$OUT_DIR" DWB_LOCK_HELD=1 bash "$TOOLS_DIR/dwb_round.sh" "${LABEL}_try$i" 9>&-
  rc=$?
  case "$rc" in
    0) echo "########## $LABEL 采用 ${LABEL}_try$i（第 $i 次尝试有效）##########"; exit 0 ;;
    3) echo "########## $LABEL 第 $i 次无效（污染轮/底盘未动/机器超售），作废重试 ##########" ;;
    4) echo "########## $LABEL 第 $i 次未跑完，作废重试 ##########" ;;
    5) echo "########## $LABEL 第 $i 次因锁冲突退出（本不该发生：锁由本脚本持有）##########" ;;
    *) echo "########## $LABEL 第 $i 次异常退出（rc=$rc），作废重试 ##########" ;;
  esac
  sleep 5
done

echo "########## $LABEL 连续 $MAX 次均未取得有效轮，各次 reason 见 <标签>_tryN/verdict.txt ##########"
exit 1
