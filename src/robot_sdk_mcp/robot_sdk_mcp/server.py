"""MCP server 入口：FastMCP 装配 + stdio 传输。

工具体一律 async + asyncio.to_thread：v1 里同步函数直接在事件循环上执行会卡死。
Robot 在 lifespan 里建、退出时停；工具经 Context 取它（不落全局）。
"""

import threading
from contextlib import asynccontextmanager

import robot_sdk
from mcp.server.fastmcp import FastMCP

from . import tools
from .config import build_config

# 启动时等动作服务端被发现的上限（秒）。rclcpp_action 不替你等 —— 不等的话第一个
# 目标会发在发现完成之前，然后一直等不到受理（实测：首个目标 30s 超时）。
SERVER_WAIT_S = 5.0


@asynccontextmanager
async def lifespan(server):
    """建 Robot、起 spin 线程、等两个动作服务端就位；退出时停。"""
    robot = robot_sdk.Robot(build_config())
    threading.Thread(target=robot.spin, daemon=True).start()
    robot.navigation().server_available(SERVER_WAIT_S)
    robot.motion().server_available(SERVER_WAIT_S)
    try:
        yield {"robot": robot}
    finally:
        robot.stop()


app = FastMCP("robot", lifespan=lifespan)

for tool in tools.TOOLS:
    app.add_tool(tool)


def main() -> None:
    """以 stdio 起服务，由 Walle 的 .agent/mcp.yaml 拉起。"""
    app.run()


if __name__ == "__main__":
    main()
