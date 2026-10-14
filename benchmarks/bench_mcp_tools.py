#!/usr/bin/env python3
"""MCP 工具调用往返基准：从 MCP 客户端发出去到拿回结果（stdio，Walle 就是这么连的）。

为什么单独测这一层：agent 侧真正"感受到"的延迟是这一整段 —— JSON-RPC over stdio + asyncio
事件循环 + to_thread 跳线 + SDK + mock 后端。目的是知道它相对 LLM 一次推理（秒级）有多小。

⚠️ 走 ./scripts/run_bench_mcp.sh（要用 robot_sdk_mcp 那个 venv 的 python，那里才有 mcp<2）。
⚠️ 不进 CI：按时序判定，进了 CI 只会多一条 flake。
"""

import argparse
import os
import platform
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_python_api as bench  # noqa: E402  复用：起 mock、查游离、分位数

import asyncio  # noqa: E402
import rclpy  # noqa: E402
from mcp import ClientSession, StdioServerParameters  # noqa: E402
from mcp.client.stdio import stdio_client  # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SERVER = os.path.join(WS, "src", "robot_sdk_mcp", "scripts", "run_server.sh")
TOOLS = (("get_status", {}), ("get_pose", {}), ("joint_states", {}), ("scan_summary", {}))


def wait_mock_ready(timeout_s=30.0):
    """等 mock 的 /robot_status 有发布者。直接问 rclpy，不在基准进程里建 SDK 客户端。"""
    rclpy.init()
    try:
        node = rclpy.create_node("bench_mcp_preflight")
        try:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.count_publishers(bench.STATUS_TOPIC) > 0:
                    return
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    raise SystemExit("mock 30 秒内没起来")


def millis_row(label, samples, note=""):
    if not samples:
        print(f"  {label:<24} {0:>6}  {'—':>8}  {'—':>8}  {'—':>8}  {note}")
        return
    print(f"  {label:<24} {len(samples):>6}  {bench.pct(samples, 0.5):>8.2f}  "
          f"{bench.pct(samples, 0.95):>8.2f}  {bench.pct(samples, 0.99):>8.2f}  {note}")


async def run_session(n):
    params = StdioServerParameters(command=SERVER, env=dict(os.environ))
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            listed = await session.list_tools()
            print(f"  服务端提供 {len(listed.tools)} 个工具：" +
                  ", ".join(tool.name for tool in listed.tools))
            print("\n  " + f"{'MCP 工具':<24} {'n':>6}  {'p50':>8}  {'p95':>8}  {'p99':>8}  (毫秒)")
            for name, args in TOOLS:
                samples, failure = [], ""
                for _ in range(n):
                    started = time.perf_counter()
                    result = await session.call_tool(name, args)
                    samples.append((time.perf_counter() - started) * 1e3)
                    if result.isError:
                        failure = str(result.content)[:120]
                        break
                millis_row(name, samples, failure or "JSON-RPC + asyncio + to_thread + SDK")
                if failure:
                    print(f"  ⚠️ 调用出错：{failure}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=200, help="每个工具的调用次数")
    args = parser.parse_args()

    print("环境：")
    print(f"  {platform.system()} / {os.cpu_count()} 核 / python {sys.version.split()[0]}")
    print(f"  CPU: {bench.cpu_model()}")
    print(f"  服务端: {SERVER}")

    bench.no_stray_robot()
    mock = subprocess.Popen([bench.MOCK_EXE, "--ros-args",
                             "-p", f"robot_description_file:={bench.ROBOT_URDF}"])
    try:
        wait_mock_ready()
        asyncio.run(run_session(args.n))
        print("\n  对照：同一个调用直连 SDK 约 4.3 µs（docs/performance.md）"
              "—— 差出来的部分就是这一层（stdio JSON-RPC + asyncio + 线程跳）的开销。")
    finally:
        mock.terminate()
        try:
            mock.wait(timeout=10)
        except subprocess.TimeoutExpired:
            mock.kill()


if __name__ == "__main__":
    main()
