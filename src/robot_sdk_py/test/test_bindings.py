"""绑定的**形状**测试：配置字段、目标构造、以及"失败抛异常"那条约定。

⚠️ 这里**不起机器人** —— 那是验收的活（`python/` 下那两个示例对着 mock 跑）。
本文件只需要能 import、字段对得上、异常语义对得上，所以很快、也不依赖 ROS 图。
"""

import pytest

import robot_sdk


def test_config_defaults_match_cpp():
    """字段名和默认值必须和 C++ 的 RobotConfig 一致（对不上就是绑定的锅）。"""
    config = robot_sdk.RobotConfig()
    assert config.node_name == "robot_sdk"
    assert config.node_namespace == ""
    assert config.use_sim_time is False

    assert config.navigation.action_name == "navigate_to_pose"
    assert config.navigation.global_frame == "map"
    assert config.motion.action_name == "move_to_joint_positions"
    assert config.motion.pose_action_name == "move_to_pose"
    assert config.motion.gripper_joint == "gripper_joint"
    # ⚠️ 这条同时钉住"默认值必须落在参考机器人夹爪行程内"（URDF 是 [0, 0.03]）。
    assert config.motion.gripper_open_position == pytest.approx(0.03)
    assert config.perception.laser_scan_topic == "scan"
    assert config.status.status_topic == "robot_status"


def test_sub_configs_are_writable():
    config = robot_sdk.RobotConfig()
    config.motion.joint_names = ["a", "b"]
    config.limits.goal_tolerance_m = 0.25
    config.navigation.arrival_tolerance_m = 0.4
    config.status.default_timeout_s = 7.5
    assert config.motion.joint_names == ["a", "b"]
    assert config.limits.goal_tolerance_m == pytest.approx(0.25)
    assert config.navigation.arrival_tolerance_m == pytest.approx(0.4)
    assert config.status.default_timeout_s == pytest.approx(7.5)


def test_goals_are_plain_objects():
    motion = robot_sdk.MotionGoal()
    assert motion.joint_names == []
    assert motion.positions == []
    assert motion.max_velocity == 0.0

    pose = robot_sdk.PoseGoal()
    assert pose.any_orientation is False
    pose.frame = "map"
    pose.target = (0.3, 0.1, 0.5, 0.0, 0.0, 0.0, 1.0)
    assert pose.frame == "map"
    assert pose.target[0] == pytest.approx(0.3)
    with pytest.raises(ValueError):
        pose.target = (1.0, 2.0)  # 少给分量要当场报错，不能静默截断

    nav = robot_sdk.NavigationGoal()
    nav.target = robot_sdk.Pose2D(1.0, 2.0, 0.5)
    assert nav.avoid_obstacles is True
    assert nav.target.yaw == pytest.approx(0.5)


def test_failure_raises_robot_sdk_error():
    """约定：失败**抛** RobotSdkError（而不是返回一个 Result 让人去查 ok()）。"""
    robot = robot_sdk.Robot(robot_sdk.RobotConfig(node_name="robot_sdk_py_test"))
    try:
        with pytest.raises(robot_sdk.RobotSdkError) as excinfo:
            robot.motion().move_to_named_target("does_not_exist")
        # 抛出来的消息里要带**稳定的错误码**，客户才能据此分支。
        assert "INVALID_ARGUMENT" in str(excinfo.value)
    finally:
        robot.stop()


def test_bad_config_raises_robot_sdk_error():
    """构造期的配置错误也必须落在 RobotSdkError 上，不能漏成 rclcpp 的 ValueError。

    `normalizeNamespace` 删掉之后，SDK 不再替调用者修正命名空间 —— 非法输入交给 rcl 拒绝，
    绑定在语言边界上把那个异常翻成 SDK 的异常。`RobotSdkError` 的基类是 `Exception`，
    所以以前的 `ValueError` 是 `except RobotSdkError:` **接不住**的。
    """
    for bad in ("/robot1/", "/a//b", "//"):
        with pytest.raises(robot_sdk.RobotSdkError) as excinfo:
            robot_sdk.Robot(
                robot_sdk.RobotConfig(node_name="bad_ns_probe", node_namespace=bad))
        assert "INVALID_ARGUMENT" in str(excinfo.value), bad

    # 节点名不合法走的是同一条路（以前它就抛，只是没人翻）
    with pytest.raises(robot_sdk.RobotSdkError) as excinfo:
        robot_sdk.Robot(robot_sdk.RobotConfig(node_name="bad name"))
    assert "INVALID_ARGUMENT" in str(excinfo.value)


def test_namespace_without_leading_slash_is_accepted():
    """rcl 本来就接受缺前导 '/' 的写法，所以 SDK 不再替它补 —— 这里钉住这条靠得住。"""
    robot = robot_sdk.Robot(robot_sdk.RobotConfig(node_name="ns_probe", node_namespace="robot1"))
    try:
        assert robot.config().node_namespace == "robot1"
    finally:
        robot.stop()
