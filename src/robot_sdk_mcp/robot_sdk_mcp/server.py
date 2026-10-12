"""MCP server 入口：FastMCP 装配 + stdio 传输。

工具体一律 async + asyncio.to_thread：SDK 的阻塞调用必须离开事件循环线程。
本包钉 mcp<2，而 v1 里同步函数直接在事件循环上执行，会卡死 server。
"""

from contextlib import asynccontextmanager

from mcp.server.fastmcp import FastMCP

from . import connection, tools


@asynccontextmanager
async def lifespan(server):
    """建连接、起 spin；进程退出时停 —— Robot 的生命周期就是这个 server 的。"""
    connection.start()
    try:
        yield
    finally:
        connection.stop()


app = FastMCP("robot", lifespan=lifespan)

for tool in tools.TOOLS:
    app.add_tool(tool)


def main() -> None:
    """以 stdio 起服务，由 Walle 的 .agent/mcp.yaml 拉起。"""
    app.run()


if __name__ == "__main__":
    main()
