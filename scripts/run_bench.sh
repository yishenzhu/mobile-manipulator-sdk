#!/usr/bin/env bash
# 跑 Python 侧性能基准：先 source ROS 与工作区，再把参数透传给基准脚本。
# ⚠️ 只用 -e pipefail：ROS 的 setup.bash 会引用未定义变量，加 -u 会当场崩（实测）。
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

exec python3 "$WS/benchmarks/bench_python_api.py" "$@"
