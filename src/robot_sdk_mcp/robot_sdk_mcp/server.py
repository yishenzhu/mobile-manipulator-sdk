"""MCP server 入口：FastMCP 装配 + stdio 传输。

⚠️ 工具体一律写成 `async def` + `anyio.to_thread.run_sync(...)`，不要写同步函数：
   SDK 的阻塞调用（go_to / move_* / wait_for_*）必须离开事件循环线程，否则整个
   server 卡死（读不了 stdin、对其它 call_tool 不响应）。
   本包与 Walle 一样钉 mcp<2，而 **v1 里同步函数是直接在事件循环上执行的**
   （不是丢线程池）—— 所以"写同步交给框架"在 v1 下是错的。见
   docs/agent-integration-plan.md §5.3。
"""

from mcp.server.fastmcp import FastMCP

app = FastMCP("robot")


def main() -> None:
    """以 stdio 起服务；客户端（Walle）通过 .agent/mcp.yaml 拉起它。"""
    app.run()


if __name__ == "__main__":
    main()
