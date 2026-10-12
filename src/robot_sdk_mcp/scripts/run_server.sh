#!/usr/bin/env bash
# Robot SDK 的 MCP server 启动器 —— 给 Agent 框架（Walle）当 MCP 子进程用。
#
# ⚠️ 不能用 `ros2 run robot_sdk_mcp robot_sdk_mcp_server`：那个入口脚本的 shebang 是
#    **系统 python3**，那里没有 mcp。所以这里显式用 venv 的 python 跑模块。
#
# 用法（Walle 的 .agent/mcp.yaml）：
#   command: bash
#   args: ["-lc", "<这个脚本的绝对路径>"]

# ⚠️ 只用 -e pipefail，**不能加 -u**：ROS 的 setup.bash 会引用未定义变量
#   （AMENT_TRACE_SETUP_FILES），nounset 会让它当场报错。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"   # scripts/ -> robot_sdk_mcp/ -> src/ -> ws 根
VENV="$WS/src/robot_sdk_mcp/.venv"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"      # 让 robot_sdk 进 PYTHONPATH

# ⚠️ 追加而不是覆盖：上面那行已经把 robot_sdk 的路径放进 PYTHONPATH 了。
export PYTHONPATH="$WS/src/robot_sdk_mcp:${PYTHONPATH:-}"

exec "$VENV/bin/python" -m robot_sdk_mcp.server
