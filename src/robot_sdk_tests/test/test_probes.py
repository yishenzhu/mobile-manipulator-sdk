"""打 **mock 机器人** 这条路的四条探针。

    pytest src/robot_sdk_tests/test/test_probes.py -s

打**适配器**那组在 test_adapter.py（两条路互斥，不能同时开 —— 见 conftest.py）。
每个探针的本体是 `check()`（不符合预期就 assert）。这里只是给它配一个 mock，
顺带让判定结果进 `colcon test-result` —— 和单元测试同一个入口。
"""

from robot_sdk_tests.probe_accel import check as check_accel
from robot_sdk_tests.probe_avoid import check as check_avoid
from robot_sdk_tests.probe_limits import check as check_limits
from robot_sdk_tests.probe_motion import check as check_motion


def test_motion_busy_and_cancel(mock_robot):
    check_motion("/move_to_joint_positions")


def test_avoid_obstacles(mock_robot):
    check_avoid("/navigate_to_pose")


def test_rejects_out_of_range(mock_robot):
    check_limits("/move_to_joint_positions")


def test_accel_is_honored(mock_robot):
    check_accel("/move_to_joint_positions")
