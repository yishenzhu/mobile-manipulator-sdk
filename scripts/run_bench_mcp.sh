#!/usr/bin/env bash
# 跑 MCP 工具往返基准：必须用 robot_sdk_mcp 那个 venv 的 python（mcp<2 装在那里）。
# ⚠️ 只用 -e pipefail：ROS 的 setup.bash 会引用未定义变量，加 -u 会当场崩（实测）。
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

exec "$WS/src/robot_sdk_mcp/.venv/bin/python" "$WS/benchmarks/bench_mcp_tools.py" "$@"
