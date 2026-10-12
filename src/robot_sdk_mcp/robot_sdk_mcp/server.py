"""MCP server 入口：FastMCP 装配 + stdio 传输。

工具体一律 async + anyio.to_thread.run_sync：SDK 的阻塞调用必须离开事件循环线程。
本包钉 mcp<2，而 v1 里同步函数直接在事件循环上执行，会卡死 server。
"""

from mcp.server.fastmcp import FastMCP

app = FastMCP("robot")


def main() -> None:
    """以 stdio 起服务，由 Walle 的 .agent/mcp.yaml 拉起。"""
    app.run()


if __name__ == "__main__":
    main()
