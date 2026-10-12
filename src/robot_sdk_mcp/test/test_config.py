"""环境变量 → RobotConfig。"""

from robot_sdk_mcp import config

ALL_ENV = (
    config.ENV_NODE_NAME,
    config.ENV_NAV_ACTION,
    config.ENV_MOTION_ACTION,
    config.ENV_POSE_ACTION,
    config.ENV_GLOBAL_FRAME,
    config.ENV_USE_SIM_TIME,
    config.ENV_JOINT_NAMES,
    config.ENV_HOME_POSITIONS,
)


def clear(monkeypatch):
    for name in ALL_ENV:
        monkeypatch.delenv(name, raising=False)


def test_defaults_match_mock_names(monkeypatch):
    clear(monkeypatch)
    cfg = config.build_config()
    assert cfg.node_name == "robot_sdk_mcp"
    # SDK 的默认动作名恰好就是 mock 的服务名 —— 所以默认配置对 mock 零设置
    assert cfg.navigation.action_name == "navigate_to_pose"
    assert cfg.motion.action_name == "move_to_joint_positions"


def test_switch_backend_by_env(monkeypatch):
    clear(monkeypatch)
    monkeypatch.setenv(config.ENV_NAV_ACTION, "sdk/navigate_to_pose")
    monkeypatch.setenv(config.ENV_USE_SIM_TIME, "true")
    cfg = config.build_config()
    assert cfg.navigation.action_name == "sdk/navigate_to_pose"
    assert cfg.use_sim_time is True


def test_joint_names_and_home_target(monkeypatch):
    clear(monkeypatch)
    monkeypatch.setenv(config.ENV_JOINT_NAMES, "shoulder_pan_joint, elbow_joint ,wrist_joint")
    monkeypatch.setenv(config.ENV_HOME_POSITIONS, "0,0.1,0.2")
    cfg = config.build_config()
    assert list(cfg.motion.joint_names) == ["shoulder_pan_joint", "elbow_joint", "wrist_joint"]
    assert dict(cfg.motion.named_targets) == {"home": [0.0, 0.1, 0.2]}


def test_empty_env_does_not_clobber_default(monkeypatch):
    clear(monkeypatch)
    monkeypatch.setenv(config.ENV_NAV_ACTION, "")
    cfg = config.build_config()
    assert cfg.navigation.action_name == "navigate_to_pose"
