#!/usr/bin/env bash
# 在 sanitizer 构建（`scripts/build_sanitizers.sh` 产出的 build-asan/install-asan）上跑核心测试，
# 并把 sanitizer 报出来的问题**原样**打出来 —— 只看摘要会漏掉根因。
#
#   ./scripts/run_sanitizer_tests.sh asan    # AddressSanitizer + UndefinedBehavior（默认）
#   ./scripts/run_sanitizer_tests.sh tsan    # ThreadSanitizer（数据竞争）
#
# 两个环境坑（都实测过）：
#   1. ROS 自身的库不是 sanitizer 版本 ⇒ 开 leak 检测会被它的噪音淹没 ⇒ 默认关掉；
#      真要查我们自己的泄漏就单独跑一个进程并把范围缩小（见 docs/memory-safety.md）。
#   2. TSan 在新内核上会 `FATAL: unexpected memory mapping`（ASLR 位数偏大）
#      ⇒ 用 `setarch -R` 关掉 ASLR 即可，不需要 root。
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-asan}"
if [ "$MODE" = "tsan" ]; then
  SAN_DIR="$WS/build-san-thread"
  INSTALL_DIR="$WS/install-san-thread"
else
  SAN_DIR="$WS/build-san-address-undefined"
  INSTALL_DIR="$WS/install-san-address-undefined"
fi
BIN_DIR="$SAN_DIR/robot_sdk_core"

source /opt/ros/jazzy/setup.bash
source "$INSTALL_DIR/setup.bash"

if [ "$MODE" = "tsan" ]; then
  # tsan.supp 只压"TSan 看不见的预编译库内部同步"（libtf2.so）产生的假阳性 —— 见该文件里的说明。
  export TSAN_OPTIONS="halt_on_error=0:second_deadlock_stack=1:suppressions=$WS/scripts/tsan.supp"
  # ⚠️ 地址空间问题有两个解法：`setarch -R`（关 ASLR）或调小 `vm.mmap_rnd_bits`（要 root）。
  #    容器里 `setarch` 可能被 seccomp 拦（personality 被禁）⇒ 先试 sysctl；成功就不用 setarch。
  LAUNCH=(setarch -R)
  if sysctl -w vm.mmap_rnd_bits=28 >/dev/null 2>&1; then
    echo "  （已 sysctl vm.mmap_rnd_bits=28 ⇒ 不需要 setarch -R）"
    LAUNCH=()
  fi
  PATTERN="WARNING: ThreadSanitizer"
else
  export ASAN_OPTIONS="detect_leaks=0:abort_on_error=0:print_stacktrace=1:halt_on_error=0"
  export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=0"
  LAUNCH=()
  PATTERN="ERROR: AddressSanitizer|runtime error:"
fi

echo "=== $MODE：$BIN_DIR ==="
failed=0
for t in "$BIN_DIR"/test_*; do
  [ -x "$t" ] || continue
  name="$(basename "$t")"
  out="$(timeout 300 "${LAUNCH[@]}" "$t" 2>&1)"
  rc=$?
  reports="$(printf '%s\n' "$out" | grep -cE "$PATTERN")"
  summary="$(printf '%s\n' "$out" | grep -E '^\[  (PASSED|FAILED)' | tail -1)"
  printf '  %-28s rc=%-4s 报告=%-3s %s\n' "$name" "$rc" "$reports" "$summary"

  # 有没有问题：sanitizer 报告、非零退出、或用例本身失败
  if [ "$reports" != "0" ] || [ "$rc" != "0" ] || printf '%s\n' "$summary" | grep -q FAILED; then
    failed=$((failed + 1))
    printf '%s\n' "$out" | grep -E "$PATTERN|^\[  FAILED|#[0-9]+ .*robot_sdk" | head -18 | sed 's/^/      /'
  fi
done

if [ "$failed" != "0" ]; then
  echo "  ✗ $failed 个测试有问题"
  exit 1
fi
echo "  ✓ 全部通过，无 sanitizer 报告"
