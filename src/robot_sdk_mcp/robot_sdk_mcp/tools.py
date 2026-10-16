"""工具：把 SDK 的四类能力包成 MCP 工具。

Robot 由 server 的 lifespan 建好，经 Context 注入 —— 不落全局。

⚠️ 这里**不能**写 `from __future__ import annotations`：v1 的 FastMCP 用
   `issubclass(param.annotation, Context)` 按**真类型**认这个参数（tools/base.py:59），
   注解一旦变成字符串就会 TypeError。
"""

import asyncio
import math

import robot_sdk
from mcp.server.fastmcp import Context

from . import errors

# "最近障碍"的默认扫描窗口：正前方 ±0.3 rad。
DEFAULT_SCAN_WINDOW = 0.3


def robot_of(ctx: Context) -> robot_sdk.Robot:
    """取 lifespan 里建好的 Robot。"""
    return ctx.request_context.lifespan_context["robot"]


async def offloaded(call):
    """阻塞的 SDK 调用丢 worker 线程，结果 / 错误翻成 envelope（否则卡死事件循环）。"""
    try:
        return errors.ok(await asyncio.to_thread(call))
    except robot_sdk.RobotSdkError as error:
        return errors.fail(error)


async def get_status(ctx: Context, timeout_s: float = 5.0, fresh: bool = False) -> dict:
    """机器人状态：state_bits（位掩码）/ allows_motion / allows_navigation / 电量 / 错误码 / 运行时长。fresh=true 走服务查询。"""
    api = robot_of(ctx).status()
    if fresh:
        return await offloaded(lambda: api.query_status(timeout_s))
    return await offloaded(lambda: api.wait_for_status(timeout_s))


async def get_pose(ctx: Context, timeout_s: float = 2.0) -> dict:
    """当前平面位姿（x, y, yaw），在配置的全局坐标系下。"""
    api = robot_of(ctx).navigation()
    return await offloaded(lambda: api.current_pose(timeout_s))


async def joint_states(ctx: Context, timeout_s: float = 5.0) -> dict:
    """等一帧关节状态（name / position）。"""
    api = robot_of(ctx).status()
    return await offloaded(lambda: api.wait_for_joint_states(timeout_s))


async def scan_summary(
    ctx: Context,
    angle_min: float = -DEFAULT_SCAN_WINDOW,
    angle_max: float = DEFAULT_SCAN_WINDOW,
    timeout_s: float = 5.0,
) -> dict:
    """等一帧激光，返回射线数与窗口内最近障碍距离（米）；无回波时 nearest_m 为 null。"""
    api = robot_of(ctx).perception()
    scan = await offloaded(lambda: api.wait_for_laser_scan(timeout_s))
    if not scan["ok"]:
        return scan
    nearest = api.nearest_obstacle_distance(angle_min, angle_max)
    return errors.ok({
        "beam_count": scan["data"]["beam_count"],
        "nearest_m": None if math.isinf(nearest) else nearest,
        "angle_window": [angle_min, angle_max],
    })


async def navigate_to(
    ctx: Context,
    x: float,
    y: float,
    yaw: float = 0.0,
    avoid_obstacles: bool = True,
    max_speed: float = 0.0,
    timeout_s: float = 60.0,
) -> dict:
    """开到 (x, y, yaw)，目标是世界坐标（不是相对底盘）。

    avoid_obstacles=false 只是"别绕路"的表达，Nav2 适配器兑现不了（它始终避障）。
    max_speed<=0 表示由后端决定。返回 success（业务层是否到达）与末距离误差。
    """
    api = robot_of(ctx).navigation()

    def goal():
        nav = robot_sdk.NavigationGoal()
        nav.target = robot_sdk.Pose2D(x, y, yaw)
        nav.avoid_obstacles = avoid_obstacles
        nav.max_speed = max_speed
        nav.timeout_s = timeout_s
        return api.go_to(nav)

    return await offloaded(goal)


async def arm_move_joints(
    ctx: Context,
    joint_names: list[str],
    positions: list[float],
    max_velocity: float = 0.0,
    timeout_s: float = 30.0,
) -> dict:
    """关节空间运动。joint_names 与 positions 一一对应；后端按名字对齐，不要求顺序一致。"""
    api = robot_of(ctx).motion()

    def goal():
        motion = robot_sdk.MotionGoal()
        motion.joint_names = joint_names
        motion.positions = positions
        motion.max_velocity = max_velocity
        motion.timeout_s = timeout_s
        return api.move_to_joint_positions(motion)

    return await offloaded(goal)


async def move_eef_to_pose(
    ctx: Context,
    x: float,
    y: float,
    z: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
    any_orientation: bool = True,
    max_velocity: float = 0.0,
    timeout_s: float = 60.0,
) -> dict:
    """末端到笛卡尔位姿 (x, y, z, qx, qy, qz, qw)；any_orientation=true 时朝向由规划器挑。

    坐标系由服务器的 ROBOT_SDK_POSE_FRAME 决定（真 MoveIt 要用它的模型帧，如 base_footprint）。
    返回 data.final_position_error（**米**）与 data.final_orientation_error（**弧度**）。
    走 SDK 的笛卡尔 action（默认 move_to_pose）—— 裸 mock 不提供它，要接了 MoveIt 适配器才有。
    """
    api = robot_of(ctx).motion()

    def goal():
        pose = robot_sdk.PoseGoal()
        pose.target = (x, y, z, qx, qy, qz, qw)
        pose.any_orientation = any_orientation
        pose.max_velocity = max_velocity
        pose.timeout_s = timeout_s
        return api.move_to_pose(pose)

    return await offloaded(goal)


async def gripper(ctx: Context, action: str = "open", position: float = 0.0) -> dict:
    """夹爪：action = open / close / set（set 用 position，米）。"""
    api = robot_of(ctx).motion()
    actions = {
        "open": api.open_gripper,
        "close": api.close_gripper,
        "set": lambda: api.set_gripper(position),
    }
    if action not in actions:
        return errors.invalid(f"action 须为 open / close / set，收到 {action!r}")
    return await offloaded(actions[action])


async def cancel(ctx: Context) -> dict:
    """急停：取消导航与运动。不进任何队列 —— 不能被忙时的动作挡住。"""
    robot = robot_of(ctx)
    failures = []
    for name, api in (("navigation", robot.navigation()), ("motion", robot.motion())):
        try:
            api.cancel_all()
        except Exception as error:  # noqa: BLE001 - 急停必须继续，不能被单个失败打断
            failures.append(f"{name}: {error}")
    return errors.ok({"cancelled": True, "failures": failures})


async def home(ctx: Context) -> dict:
    """回 home 位（SDK 配置里的具名目标 "home"）。未配置会返回 INVALID_ARGUMENT。"""
    api = robot_of(ctx).motion()
    return await offloaded(lambda: api.home())


async def named_target(ctx: Context, name: str) -> dict:
    """去 SDK 配置里声明的具名位姿（如 home）。名字没配置则 INVALID_ARGUMENT。"""
    api = robot_of(ctx).motion()
    return await offloaded(lambda: api.move_to_named_target(name))


TOOLS = (
    get_status,
    get_pose,
    joint_states,
    scan_summary,
    navigate_to,
    arm_move_joints,
    move_eef_to_pose,
    gripper,
    cancel,
    home,
    named_target,
)
