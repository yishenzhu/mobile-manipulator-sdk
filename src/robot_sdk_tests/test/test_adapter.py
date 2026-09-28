"""打**适配器**那条路（假下游 + motion_adapter）。

和 test_probes.py 是两条路：那边打 mock 机器人，这边打适配器。
两边**不能同时开**（同名服务端会打架 —— conftest.py 会直接拦下来）。

    pytest src/robot_sdk_tests/test/test_adapter.py -s

`probe_avoid` 不在这一组 —— 适配器只管关节运动，导航那条链路不在 `motion_chain` 里。
"""

from robot_sdk_tests.probe_accel import check as check_accel
from robot_sdk_tests.probe_motion import check as check_motion


def test_motion_via_adapter(motion_chain):
    check_motion(motion_chain)


def test_accel_via_adapter(motion_chain):
    check_accel(motion_chain)
