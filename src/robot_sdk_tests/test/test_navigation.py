"""打 **Nav2 整栈** 那条路（mock 机器人 + nav2_adapter + Nav2 + map_server）。

    pytest src/robot_sdk_tests/test/test_navigation.py -s

和另两组互斥 —— 三条路靠 `scope="module"` 错开（见 conftest.py）。
这条最慢：起整栈 + 两次导航，跑一轮几十秒。
"""

from robot_sdk_tests.probe_navigate import check as check_navigate


def test_reaches_goal_and_cancels(navigation):
    check_navigate(navigation)
