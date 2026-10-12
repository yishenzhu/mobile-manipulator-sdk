"""进程级 Robot 单例：一次连接，server 全生命周期复用。

Robot 与 spin 线程的生命周期 = server 进程的生命周期。别每个工具调用建一次
（会重复建节点、重复起执行器）。
"""

from __future__ import annotations

import threading

import robot_sdk

from .config import build_config

ROBOT: robot_sdk.Robot | None = None


def start() -> None:
    """建 Robot 并起 spin 线程（spin 放开 GIL，后台持续处理回调）。"""
    global ROBOT
    ROBOT = robot_sdk.Robot(build_config())
    threading.Thread(target=ROBOT.spin, daemon=True).start()


def robot() -> robot_sdk.Robot:
    if ROBOT is None:
        raise RuntimeError("robot 未启动")
    return ROBOT


def stop() -> None:
    if ROBOT is not None:
        ROBOT.stop()
