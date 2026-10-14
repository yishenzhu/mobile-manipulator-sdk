#!/usr/bin/env python3
"""SDK Python 侧基准：从 Python 计时，打印 p50/p95/p99（微秒）。走 ./scripts/run_bench.sh。

量的是**从 Python 调进来到拿到值**这一整段（含 pybind 的投影与数据拷贝）—— agent/MCP 那侧
感受到的就是这一段。后端一律用 mock：量的是 **SDK 自身开销**，不是 Nav2/MoveIt 的耗时。

⚠️ 不进 CI：缓存读与动作都按时序判定，进 CI 只会多一条 flake（见 src/robot_sdk_tests/README.md）。
"""

import argparse
import json
import os
import platform
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory

import robot_sdk

MOCK_EXE = os.path.join(get_package_prefix("robot_sdk_sim"), "lib", "robot_sdk_sim",
                        "mock_robot_node")
ROBOT_URDF = os.path.join(get_package_share_directory("robot_description"),
                          "urdf", "mobile_manipulator.urdf")
STATUS_TOPIC = "/robot_status"

ROWS = []


def no_stray_robot():
    """起跑前确认没有游离的机器人 —— 有第二个同名服务端时，数字会是垃圾（README 那节）。"""
    rclpy.init()
    try:
        node = rclpy.create_node("bench_preflight")
        try:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.count_publishers(STATUS_TOPIC) > 0:
                    raise SystemExit(f"{STATUS_TOPIC} 已有发布者 —— 先清掉游离的机器人再跑基准")
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def measure(call, n):
    """跑 n 次 call（零参可调用）。计时区间 = **一次完整用户写法**，含 robot.status() 这类
    属性访问 —— 与动作那边同一口径，别再出现"一边含、一边不含"。"""
    samples = []
    for _ in range(n):
        started = time.perf_counter()
        call()
        samples.append((time.perf_counter() - started) * 1e6)
    return samples


def pct(samples, quantile):
    if not samples:
        return float("nan")
    ordered = sorted(samples)
    return ordered[min(int(quantile * len(ordered)), len(ordered) - 1)]


def row(label, samples, note=""):
    ROWS.append({"item": label, "n": len(samples), "p50_us": round(pct(samples, 0.5), 2),
                 "p95_us": round(pct(samples, 0.95), 2), "p99_us": round(pct(samples, 0.99), 2),
                 "note": note})
    if not samples:
        print(f"  {label:<32} {0:>6}  {'—':>8}  {'—':>8}  {'—':>8}  {note}")
        return
    print(f"  {label:<32} {len(samples):>6}  {pct(samples, 0.5):>8.2f}  "
          f"{pct(samples, 0.95):>8.2f}  {pct(samples, 0.99):>8.2f}  {note}")


def move_samples(robot, names, origin, delta, count):
    """跑 count 次动作，返回 (每轮微秒, 第一个错误)。后端拒收时**不中断**整个基准。"""
    samples, failure = [], ""
    for index in range(count):
        goal = robot_sdk.MotionGoal()
        goal.joint_names = names
        step = delta if index % 2 == 0 else -delta
        goal.positions = [origin.get(name, 0.0) + step for name in names]
        goal.max_velocity = 1.0
        # 计时区间与缓存读同一口径：都含 robot.motion()/robot.status() 这次属性访问
        # （goal 的构造在区间外 —— 那是用户准备数据，不属于"调用"）。
        started = time.perf_counter()
        try:
            robot.motion().move_to_joint_positions(goal)
        except robot_sdk.RobotSdkError as exc:
            failure = str(exc)
            break
        samples.append((time.perf_counter() - started) * 1e6)
    return samples, failure


def loop_iterations(seconds):
    """纯 Python 循环的吞吐。spin 若**没**放开 GIL，这里会直接卡死（见绑定里的 call_guard）。"""
    count, started = 0, time.perf_counter()
    while time.perf_counter() - started < seconds:
        count += 1
    return count


def cpu_model():
    try:
        with open("/proc/cpuinfo") as handle:
            for line in handle:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=20000, help="缓存读的样本数")
    parser.add_argument("--moves", type=int, default=15, help="动作端到端的次数")
    parser.add_argument("--json", help="把结果另存一份 JSON（留档用）")
    args = parser.parse_args()

    print("环境：")
    print(f"  {platform.system()} {platform.release()} / {platform.machine()} / "
          f"{os.cpu_count()} 核 / python {platform.python_version()}")
    print(f"  CPU: {cpu_model()}")
    print(f"  ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(未设)')}")

    no_stray_robot()
    baseline = loop_iterations(0.5)

    mock = subprocess.Popen([MOCK_EXE, "--ros-args", "-p", f"robot_description_file:={ROBOT_URDF}"])
    robot = robot_sdk.Robot()
    spin = threading.Thread(target=robot.spin, daemon=True)
    spin.start()
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if robot.status().has_status() and robot.perception().has_laser_scan():
                break
            time.sleep(0.05)
        else:
            raise SystemExit("mock 30 秒内没就绪（robot_status / scan）")

        print("\n  " + f"{'项目':<30} {'n':>6}  {'p50':>8}  {'p95':>8}  {'p99':>8}  (微秒)")
        row("has_status（pybind 地板）", measure(lambda: robot.status().has_status(), args.n))
        row("status()", measure(lambda: robot.status().status(), args.n))
        row("joint_states()（3×6）", measure(lambda: robot.status().joint_states(), args.n))
        row("laser_scan()（360 线）",
            measure(lambda: robot.perception().laser_scan(), args.n // 4), "含 ranges 列表拷贝")
        row("nearest_obstacle_distance()",
            measure(lambda: robot.perception().nearest_obstacle_distance(), args.n))

        states = robot.status().joint_states()
        origin = dict(zip(states["name"], states["position"]))
        # ⚠️ 不能用 config().motion.joint_names：它的默认是**空**（真名从 ROS 参数来），而基准是
        #    直接构造 Robot 不走参数 ⇒ 从 /joint_states 取名，并排掉夹爪（URDF 里只有 [0,0.03]，
        #    跟着动会被 mock 判超限而拒收 —— 实测踩过）。
        gripper = robot.config().motion.gripper_joint
        names = [name for name in states["name"] if name != gripper]
        for delta, note in ((0.0, "原地：纯动作链路开销"),
                            (0.02, "0.02 rad：含 mock 的运动时间")):
            moves, failure = move_samples(robot, names, origin, delta, args.moves)
            row(f"move_to_joint_positions({delta:g})", moves, failure or note)
            if failure:
                print(f"  ⚠️ 动作被拒收：{failure}")

        with_spin = loop_iterations(0.5)
        print(f"\nGIL：纯 Python 循环 0.5 s —— 无 SDK {baseline} 次 / spin 在转 {with_spin} 次"
              f"（比值 {with_spin / baseline:.2f}；若 GIL 没放开这里会卡死）")
    finally:
        robot.stop()
        spin.join(timeout=5.0)
        mock.terminate()
        try:
            mock.wait(timeout=10)
        except subprocess.TimeoutExpired:
            mock.kill()

    if args.json:
        with open(args.json, "w") as handle:
            json.dump({"cpu": cpu_model(), "cores": os.cpu_count(), "rows": ROWS}, handle,
                      ensure_ascii=False, indent=2)
        print(f"\n  结果已存: {args.json}")


if __name__ == "__main__":
    main()
