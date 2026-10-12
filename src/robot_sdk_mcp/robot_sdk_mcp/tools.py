"""只读工具：状态 / 位姿 / 关节 / 障碍。"""

from __future__ import annotations

import asyncio
import math

import robot_sdk

from . import connection, errors

# "最近障碍"的默认扫描窗口：正前方 ±0.3 rad。
DEFAULT_SCAN_WINDOW = 0.3


async def offloaded(call):
    """把阻塞的 SDK 调用丢到 worker 线程，结果 / 错误翻成 envelope。

    一律走它：SDK 调用会阻塞当前线程几十秒，直接在事件循环上跑会卡死整个 server。
    """
    try:
        return errors.ok(await asyncio.to_thread(call))
    except robot_sdk.RobotSdkError as error:
        return errors.fail(error)


async def get_status(timeout_s: float = 5.0, fresh: bool = False) -> dict:
    """读机器人状态：state / 电量 / 充电 / 急停 / 错误码 / 运行时长。fresh=true 走服务查询。"""
    api = connection.robot().status()
    if fresh:
        return await offloaded(lambda: api.query_status(timeout_s))
    return await offloaded(lambda: api.wait_for_status(timeout_s))


async def get_pose(timeout_s: float = 2.0) -> dict:
    """当前平面位姿（x, y, yaw），在配置的全局坐标系下。"""
    return await offloaded(lambda: connection.robot().navigation().current_pose(timeout_s))


async def joint_states(timeout_s: float = 5.0) -> dict:
    """等一帧关节状态（name / position）。"""
    return await offloaded(lambda: connection.robot().status().wait_for_joint_states(timeout_s))


async def scan_summary(
    angle_min: float = -DEFAULT_SCAN_WINDOW,
    angle_max: float = DEFAULT_SCAN_WINDOW,
    timeout_s: float = 5.0,
) -> dict:
    """等一帧激光，返回射线数与窗口内最近障碍距离（米）；无回波时 nearest_m 为 null。"""
    api = connection.robot().perception()
    scan = await offloaded(lambda: api.wait_for_laser_scan(timeout_s))
    if not scan["ok"]:
        return scan
    nearest = api.nearest_obstacle_distance(angle_min, angle_max)
    return errors.ok({
        "beam_count": scan["data"]["beam_count"],
        "nearest_m": None if math.isinf(nearest) else nearest,
        "angle_window": [angle_min, angle_max],
    })
