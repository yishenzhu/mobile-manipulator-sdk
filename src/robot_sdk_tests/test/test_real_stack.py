"""真栈那条路：真 ros2_control + 真 move_group（`real_stack` fixture 起的三个进程）。
它验**集成** —— MoveIt 认不认我们翻的约束、控制器能不能真把轨迹走完；替身那条
（test_pose.py）验的是"翻译对不对"。两边各抓各的，都要有。"""

from robot_sdk_tests.probe_pose_real import check as check_pose_real


def test_pose_real_stack(real_stack):
    check_pose_real(real_stack)
