"""笛卡尔那条路：适配器把 SDK 的「位姿」翻成 MoveGroup 的「约束」没有。

打的是 `pose_chain.launch.py` 起来的那一对（假 move_group + moveit_adapter）。
**不做规划** —— 规划是 MoveIt 的活，那个替身不干，所以这里也不验。
"""

from robot_sdk_tests.probe_pose import check as check_pose


def test_pose_translation(pose_chain):
    check_pose(pose_chain)
