"""三条路：mock 机器人、假下游 + 适配器、Nav2 整栈。

起不来 = fixture 报错 = `colcon test-result` 里的 **error**；
判据不对 = assert 失败 = 那里的 **failure**。两类天生分开，CI 一眼看得出
是"环境没起好"还是"行为不对"。

**三条路互斥**，靠 `scope="module"` 保证：每个文件各起一份、**拆干净了才走下一个
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
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from robot_sdk_interfaces.action import MoveToJointPositions, NavigateToPose

MOCK_EXE = os.path.join(get_package_prefix("robot_sdk_sim"), "lib", "robot_sdk_sim",
                        "mock_robot_node")
# mock 的关节限位来自这份 URDF —— 它就是"机器人自述"，所以 fixture 得喂给它。
# 不喂的话 mock 不认识限位（那正是以前的行为：超限目标照发照转）。
ROBOT_URDF = os.path.join(get_package_share_directory("robot_description"),
                          "urdf", "mobile_manipulator.urdf")
ADAPTER_MOTION = "/sdk/move_to_joint_positions"
ADAPTER_NAVIGATION = "/sdk/navigate_to_pose"


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

            costmaps = []
            node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                     costmaps.append, 10)
            deadline = time.monotonic() + 60.0
            while not costmaps and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            assert costmaps, f"等不到 /global_costmap/costmap —— Nav2 还没激活（日志：{log.name}）"
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
