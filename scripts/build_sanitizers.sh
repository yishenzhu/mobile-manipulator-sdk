#!/usr/bin/env bash
# 用 sanitizer 构建到一个**独立的工作空间**（默认 build-asan / install-asan），不动日常构建。
#
#   ./scripts/build_sanitizers.sh                 # ASan + UBSan（默认）
#   ./scripts/build_sanitizers.sh address          # 只 ASan（含 LSan）
#   ./scripts/build_sanitizers.sh thread           # TSan（数据竞争）
#
# 为什么独立目录：sanitizer 版本与日常版本混在一个 build/ 里会互相覆盖，
# 排查完还得重建回来 —— 分开之后互不干扰（`.gitignore` 里已忽略 `build-*` / `install-*`）。
#
# 跑测试：
#   ASAN_OPTIONS=detect_leaks=0 ./build-asan/robot_sdk_core/test_robot_lifecycle
#   （ROS 自身的库不是 sanitizer 版本，leak 检测会产生大量噪音，排查我们自己的代码时先关掉）
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SAN="${1:-address,undefined}"
# ⚠️ 每种 sanitizer 用**各自的**目录：ASan 与 TSan 不能共用同一份产物
#    （混在一起会互相覆盖，跑起来只会看到 "unexpected memory mapping" 之类的假故障 —— 实测踩过）。
SUFFIX="$(printf '%s' "$SAN" | tr ',' '-' | tr -d ' ')"
BUILD_BASE="$WS/build-san-$SUFFIX"
INSTALL_BASE="$WS/install-san-$SUFFIX"
PKGS="${2:-robot_sdk_core}"

source /opt/ros/jazzy/setup.bash

echo "  sanitizer: $SAN"
echo "  包:        $PKGS"
echo "  构建到:    $BUILD_BASE / $INSTALL_BASE"

colcon build \
  --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE" \
  --packages-up-to $PKGS \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    "-DCMAKE_CXX_FLAGS=-fsanitize=$SAN -fno-omit-frame-pointer -g" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=$SAN" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=$SAN"
