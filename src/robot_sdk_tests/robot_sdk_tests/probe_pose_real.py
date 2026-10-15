#!/usr/bin/env python3
"""验收探针：真栈 —— 真 ros2_control + 真 move_group，验 6 DOF 解得出**带姿态**的目标。
seed 里腕关节非零 ⇒ 朝向非平凡 ⇒ any_orientation=False 成功即说明 KDL 解得出 6 维位姿。
少一个腕关节就退化成欠自由度、这条必失败（-27），所以它同时钉住了"腕部那两段还在"。"""

import math

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_sdk_interfaces.action import MoveToPose

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint",
       "wrist_pitch_joint", "wrist_roll_joint"]
EE = "gripper_base_link"
FRAME = "base_footprint"

# 适配器默认的**每轴**姿态容差（moveit_adapter 的 orientation_tolerance 参数）。
ORIENTATION_TOLERANCE = 0.05
# ⚠️ final_orientation_error 是**合成**转角，不是"每轴"那个数：三轴各偏到容差上限时，
#    合成的轴角上界是 √3 × 每轴容差。拿"每轴 0.05"直接卡合成角会误报（实测踩过）。
ORIENTATION_BOUND = math.sqrt(3.0) * ORIENTATION_TOLERANCE

# 三个腕关节都非零 —— 这是关键：朝向非平凡。
SEEDS = [
    [0.30, 0.50, 1.00, 0.00, 0.50, 0.60],
    [0.60, 0.70, -0.40, -0.50, 0.80, -0.90],
]


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_pose_real")
        self.client = ActionClient(self, MoveToPose, action_name)
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.sv_client = self.create_client(GetStateValidity, "/check_state_validity")

    def wait(self, future, timeout=240.0):
        # 必须带超时：`spin_until_future_complete` 超时后返回，但 `result()` 是阻塞的。
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()

    @staticmethod
    def state(seed):
        state = RobotState()
        state.joint_state.name = ARM
        state.joint_state.position = [float(p) for p in seed]
        return state


def check(action_name="/sdk/move_to_pose"):
    """带姿态的可达位姿该走得到。不符合预期就 assert。"""
    rclpy.init()
    try:
        probe = Probe(action_name)
        assert probe.client.wait_for_server(timeout_sec=30.0), f"等不到 {action_name}"
        assert probe.fk_client.wait_for_service(timeout_sec=30.0), "等不到 /compute_fk"
        assert probe.sv_client.wait_for_service(timeout_sec=30.0), \
            "等不到 /check_state_validity"

        for index, seed in enumerate(SEEDS, 1):
            # ① 自碰撞检查 —— 腕部那两段之间不该被误判成碰撞（SRDF 白名单对不对）。
            sv = probe.wait(probe.sv_client.call_async(
                GetStateValidity.Request(robot_state=probe.state(seed), group_name="arm")))
            pairs = [(c.contact_body_1, c.contact_body_2) for c in sv.contacts]
            assert sv.valid, f"seed {index} 被判成自碰撞：{pairs}"

            # ② FK 取那个位姿 —— 它**必然可达**，所以失败只能怪自由度/配置，不能怪"够不着"。
            request = GetPositionFK.Request()
            request.header.frame_id = FRAME
            request.fk_link_names = [EE]
            request.robot_state = probe.state(seed)
            fk = probe.wait(probe.fk_client.call_async(request))
            assert fk.pose_stamped, f"seed {index}：/compute_fk 没给出位姿"
            pose = fk.pose_stamped[0]

            # ③ 真填姿态约束地发一次。
            goal = MoveToPose.Goal()
            goal.target_pose = pose
            goal.any_orientation = False
            goal.max_velocity = 0.2
            handle = probe.wait(probe.client.send_goal_async(goal), timeout=30.0)
            assert handle.accepted, f"seed {index}：目标该被接受"
            result = probe.wait(handle.get_result_async()).result

            p = pose.pose.position
            print(f"  seed{index} ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                  f"success={result.success} 位置误差={result.final_position_error:.4f} "
                  f"姿态误差={result.final_orientation_error:.4f}  '{result.message}'")
            assert result.success, f"seed {index} 该成功到达（'{result.message}'）"
            assert result.final_orientation_error <= ORIENTATION_BOUND, \
                (f"seed {index} 姿态误差 {result.final_orientation_error:.4f} "
                 f"超出容忍带上界 {ORIENTATION_BOUND:.4f}")

        print("✅ 真栈：6 DOF 解得出带姿态的目标")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    check()
