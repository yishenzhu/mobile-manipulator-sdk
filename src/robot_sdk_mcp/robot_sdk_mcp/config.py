"""从环境变量构造 RobotConfig（Walle 的 mcp.yaml 以 env: 注入）。

action 名随后端切换：mock 用 navigate_to_pose 等，接适配器换 sdk/ 前缀
（同名不同类型的 action server 无法共存）。ROS_DOMAIN_ID 不在此读，rcl 直接取环境。
"""

from __future__ import annotations

import os

import robot_sdk

ENV_NODE_NAME = "ROBOT_SDK_NODE_NAME"
ENV_NAV_ACTION = "ROBOT_SDK_NAV_ACTION"
ENV_MOTION_ACTION = "ROBOT_SDK_MOTION_ACTION"
ENV_POSE_ACTION = "ROBOT_SDK_POSE_ACTION"
ENV_GLOBAL_FRAME = "ROBOT_SDK_GLOBAL_FRAME"
ENV_USE_SIM_TIME = "ROBOT_SDK_USE_SIM_TIME"
ENV_JOINT_NAMES = "ROBOT_SDK_JOINT_NAMES"
ENV_HOME_POSITIONS = "ROBOT_SDK_HOME_POSITIONS"


def build_config() -> robot_sdk.RobotConfig:
    """读环境变量装配配置，未设置的项保留 SDK 默认值。"""
    config = robot_sdk.RobotConfig(
        node_name=os.environ.get(ENV_NODE_NAME, "robot_sdk_mcp"),
        use_sim_time=env_bool(ENV_USE_SIM_TIME, default=False),
    )
    set_if_env(config.navigation, "action_name", ENV_NAV_ACTION)
    set_if_env(config.navigation, "global_frame", ENV_GLOBAL_FRAME)
    set_if_env(config.motion, "action_name", ENV_MOTION_ACTION)
    set_if_env(config.motion, "pose_action_name", ENV_POSE_ACTION)
    # 具名目标（home）只能代码配置：named_targets 是 map，ROS 参数没有 map 类型。
    joint_names = env_list(ENV_JOINT_NAMES)
    if joint_names:
        config.motion.joint_names = joint_names
    home = env_floats(ENV_HOME_POSITIONS)
    if home:
        config.motion.named_targets = {"home": home}
    return config


def env_list(env_name: str) -> list[str]:
    """逗号分隔 → 列表；未设置返回空。"""
    raw = os.environ.get(env_name)
    return [item.strip() for item in raw.split(",") if item.strip()] if raw else []


def env_floats(env_name: str) -> list[float]:
    """逗号分隔 → 浮点列表。"""
    return [float(item) for item in env_list(env_name)]


def set_if_env(section, field: str, env_name: str) -> None:
    """有环境变量才覆盖，避免空串打掉默认值。"""
    value = os.environ.get(env_name)
    if value:
        setattr(section, field, value)


def env_bool(env_name: str, default: bool) -> bool:
    raw = os.environ.get(env_name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")
