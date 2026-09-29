"""四条路：mock 机器人、假下游 + 适配器、真栈（ros2_control + move_group）、Nav2 整栈。

起不来 = fixture 报错 = `colcon test-result` 里的 **error**；
判据不对 = assert 失败 = 那里的 **failure**。两类天生分开，CI 一眼看得出
是"环境没起好"还是"行为不对"。

**四条路互斥**，靠 `scope="module"` 保证：每个文件各起一份、**拆干净了才走下一个
文件**，所以它们在时间上不重叠。这一点别动 —— 改成 `session` 级的话，几个会同时
活到会话末尾，而**同名 action 服务端不止一个**时，回应会串、future 可能永远不完成
（README 开头那节）。
"""

import os
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from robot_sdk_interfaces.action import MoveToJointPositions, MoveToPose, NavigateToPose

ADAPTER_POSE = "/sdk/move_to_pose"

MOCK_EXE = os.path.join(get_package_prefix("robot_sdk_sim"), "lib", "robot_sdk_sim",
                        "mock_robot_node")
# mock 的关节限位来自这份 URDF —— 它就是"机器人自述"，所以 fixture 得喂给它。
# 不喂的话 mock 不认识限位（那正是以前的行为：超限目标照发照转）。
ROBOT_URDF = os.path.join(get_package_share_directory("robot_description"),
                          "urdf", "mobile_manipulator.urdf")
ADAPTER_MOTION = "/sdk/move_to_joint_positions"
ADAPTER_NAVIGATION = "/sdk/navigate_to_pose"

# 真栈那条路要的包。
REAL_STACK_PACKAGES = ("controller_manager", "ros2_control", "moveit_ros_move_group",
                       "moveit_configs_utils")


@pytest.fixture(scope="module")
def real_stack():
    """起真栈：真 ros2_control + 真 move_group + 适配器，返回适配器的动作名。
    ⚠️ 四条路里唯一能验"真 MoveIt 认不认"的；替身那条验的是"我们翻得对不对"，两者互补。"""
    # 缺包就**明确跳过并说清缺了什么** —— 不静默 skip：静默跳过会让"没装 ros2_control
    # 的机器"看起来跟"跑过了"一样绿。
    missing = []
    for name in REAL_STACK_PACKAGES:
        try:
            get_package_prefix(name)
        except Exception:
            missing.append(name)
    if missing:
        pytest.skip("真栈那条路需要这些包，本机缺：" + ", ".join(missing))

    log = tempfile.NamedTemporaryFile(prefix="real_stack_", suffix=".log", delete=False)
    procs = []
    try:
        # 顺序有意义：先执行侧、后规划侧 —— move_group 启动时就要去连控制器。
        procs.append(subprocess.Popen(
            ["ros2", "launch", "robot_description", "ros2_control.launch.py"],
            start_new_session=True, stdout=log, stderr=subprocess.STDOUT))
        procs.append(subprocess.Popen(
            ["ros2", "launch", "robot_description", "move_group.launch.py"],
            start_new_session=True, stdout=log, stderr=subprocess.STDOUT))
        procs.append(subprocess.Popen(
            ["ros2", "run", "robot_sdk_bridge", "moveit_adapter", "--ros-args",
             "-p", "sdk_action_name:=sdk/move_to_pose",
             "-p", "move_group_action_name:=/move_action",
             "-p", "planning_group:=arm",
             "-p", "end_effector_link:=gripper_base_link"],
            start_new_session=True, stdout=log, stderr=subprocess.STDOUT))

        rclpy.init()
        try:
            node = rclpy.create_node("wait_for_real_stack")

            # 就绪判据①：控制器真的到 active。**不是**"服务在"，也**不是**"/joint_states 在发"
            # —— 光有广播器也能发关节状态。这条是最容易假绿的那个（见 README 那节）。
            client = node.create_client(ListControllers,
                                       "/controller_manager/list_controllers")
            assert client.wait_for_service(timeout_sec=120.0), \
                f"等不到 /controller_manager/list_controllers（日志：{log.name}）"
            want = {"arm_controller", "joint_state_broadcaster"}
            active, deadline = set(), time.monotonic() + 120.0
            while time.monotonic() < deadline:
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if future.done() and future.result() is not None:
                    active = {c.name for c in future.result().controller
                              if c.state == "active"}
                    if want <= active:
                        break
                time.sleep(0.2)
            assert want <= active, \
                (f"控制器没到 active（当前 {sorted(active) or '无'}，日志：{log.name}）")

            adapter = ActionClient(node, MoveToPose, ADAPTER_POSE)
            assert adapter.wait_for_server(timeout_sec=120.0), \
                f"等不到 {ADAPTER_POSE}（日志：{log.name}）"

            # 就绪判据②：TF 上查得到末端。结果里的最终误差就是从它量出来的 ——
            # 查不到会留成 0，测试就变成假绿（适配器会改成报一句话）。
            buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(buffer, node)
            ok, deadline = False, time.monotonic() + 60.0
            while time.monotonic() < deadline and not ok:
                rclpy.spin_once(node, timeout_sec=0.2)
                try:
                    buffer.lookup_transform("base_footprint", "gripper_base_link",
                                            rclpy.time.Time())
                    ok = True
                except Exception:
                    pass
            assert ok, \
                f"TF 上查不到 base_footprint→gripper_base_link（日志：{log.name}）"
        finally:
            rclpy.shutdown()

        yield ADAPTER_POSE
    finally:
        # 拆卸不许抛：抛了报告里多一条 error，进程还可能没死。SIGINT 整组 + KILL 兜底。
        for proc in reversed(procs):
            if proc.poll() is not None:
                continue
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        log.close()


@pytest.fixture(scope="module")
def mock_robot():
    """起一个 mock 机器人，供 test_probes.py 的四条共用。

    **直接起可执行文件**，不要 `ros2 run` —— 那样 `$!` 拿到的是 Python 壳的 PID，
    杀壳杀不掉里面的进程（前几轮"清理干净"全是自欺）。
    """
    proc = subprocess.Popen(
        [MOCK_EXE, "--ros-args", "-p", f"robot_description_file:={ROBOT_URDF}"],
    )
    try:
        yield
    finally:
        # 拆卸不许抛：抛了 `colcon test-result` 里会多一条 error，而且进程可能还没死。
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()          # 杀不动就下狠手
            proc.wait(timeout=5)


@pytest.fixture(scope="module")
def motion_chain():
    """起"假下游 + 适配器"，返回适配器那个动作名，供 test_adapter.py 用。"""
    # 自成进程组：`ros2 launch` 底下还挂着假下游和适配器，收尾时要**打这一组** ——
    # 只 kill 它自己的话，孩子们会变孤儿，就是那个"游离机器人"的来路。
    proc = subprocess.Popen(["ros2", "launch", "robot_sdk_tests",
                             "motion_chain.launch.py"], start_new_session=True)
    try:
        # 等到"能干活"为止，而不只是"服务端在"：适配器要靠 /joint_states 拿起点，
        # 没收到就**拒收目标**。只等 wait_for_server 会偶发假失败。链路起来约 8 秒，
        # 所以这个活归 fixture 管，别让 `check` 里那 10 秒背这个锅。
        rclpy.init()
        try:
            node = rclpy.create_node("wait_for_adapter")
            client = ActionClient(node, MoveToJointPositions, ADAPTER_MOTION)
            assert client.wait_for_server(timeout_sec=40.0), f"等不到 {ADAPTER_MOTION}"

            states = []
            node.create_subscription(JointState, "/joint_states", states.append, 10)
            deadline = time.monotonic() + 20.0
            while not states and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert states, "等不到 /joint_states —— 适配器要靠它拿起点"
        finally:
            rclpy.shutdown()

        yield ADAPTER_MOTION
    finally:
        # SIGINT 而不是 kill：`ros2 launch` 是进程管理器，收到 SIGINT 会把子进程
        # 一起带走（实测过）。上面那条 killpg 才是兜底 —— 别指望它先走到。
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)


@pytest.fixture(scope="module")
def pose_chain():
    """起"假 move_group + 适配器"，返回适配器那个动作名，供 test_pose.py 用。"""
    # 自成进程组（理由和 motion_chain 一样：只 kill 自己会把孩子变孤儿）。
    proc = subprocess.Popen(["ros2", "launch", "robot_sdk_tests",
                             "pose_chain.launch.py"], start_new_session=True)
    try:
        # 这条链路**没有**那种"服务端在了但还不能干活"的前置：适配器构造好就 advertise，
        # 假 move_group 也是，不需要等任何数据流（motion_chain 要等 /joint_states，
        # 导航要等生命周期 —— 那两条才需要额外判据）。
        rclpy.init()
        try:
            node = rclpy.create_node("wait_for_moveit_adapter")
            client = ActionClient(node, MoveToPose, ADAPTER_POSE)
            assert client.wait_for_server(timeout_sec=40.0), f"等不到 {ADAPTER_POSE}"
        finally:
            rclpy.shutdown()

        yield ADAPTER_POSE
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)


@pytest.fixture(scope="module")
def navigation():
    """起 Nav2 整栈（mock + nav2_adapter + Nav2 + map_server），返回适配器的动作名。"""
    # 整栈有十来个节点，日志能把 pytest 的输出淹掉 —— 写文件里，出事了看消息里的路径。
    log = tempfile.NamedTemporaryFile(prefix="nav2_launch_", suffix=".log", delete=False)
    proc = subprocess.Popen(["ros2", "launch", "robot_sdk_nav2", "navigation.launch.py"],
                            start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
    try:
        # 等到"真能导航"为止，而不是"服务端在"：动作服务端在节点构造时就 advertise
        # 了，**不代表生命周期激活完成**。代价地图是 planner/controller 激活之后才发的，
        # 拿它当"能干活"的信号。（mock_robot 那次栽在同一类假设上。）
        rclpy.init()
        try:
            node = rclpy.create_node("wait_for_nav2")
            client = ActionClient(node, NavigateToPose, ADAPTER_NAVIGATION)
            assert client.wait_for_server(timeout_sec=60.0), \
                f"等不到 {ADAPTER_NAVIGATION}（日志：{log.name}）"

            # ⚠️ 判据必须问**生命周期**，不能看代价地图有没有帧：那帧是 latch 的，早于全栈
            # 就绪就存在 ⇒ 用它会让探针提前放行，撞上还在 activating 的 bt_navigator
            # （"Action server is inactive"）⇒ 整栈 abort。判据就是 bt_navigator 到 active。
            state_client = node.create_client(GetState, "/bt_navigator/get_state")
            assert state_client.wait_for_service(timeout_sec=60.0), \
                f"等不到 /bt_navigator/get_state（日志：{log.name}）"
            deadline = time.monotonic() + 90.0
            state = ""
            while time.monotonic() < deadline:
                future = state_client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if future.done() and future.result() is not None:
                    state = future.result().current_state.label
                    if state == "active":
                        break
                time.sleep(0.2)
            assert state == "active", \
                f"bt_navigator 没到 active（当前 {state or '未知'}，日志：{log.name}）"
        finally:
            rclpy.shutdown()

        yield ADAPTER_NAVIGATION
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        log.close()
