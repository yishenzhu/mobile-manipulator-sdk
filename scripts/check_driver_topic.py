#!/usr/bin/env python3
"""给一个"驱动话题"做接入体检。

为什么要有它：驱动接入踩的坑几乎都是**静默**的 —— QoS 不兼容就一条数据都收不到；
时钟源不一致就 TF 报错或数据"来自未来"；frame_id 拼错就整条 TF 链断掉。
逐条手工查很慢，所以做成一条命令。

    python3 scripts/check_driver_topic.py /imu
    python3 scripts/check_driver_topic.py /scan --expect-hz 10 --target-frame base_footprint
    python3 scripts/check_driver_topic.py /odom --window 5

退出码：出现 ✗ 时非 0（能直接当自检/门禁用）。
"""

import argparse
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message

import tf2_ros

OK, WARN, BAD = "✓", "⚠️", "✗"
results = []


def report(level, name, detail=""):
    results.append(level)
    print(f"  {level} {name}" + (f"：{detail}" if detail else ""))


def duration_to_sec(d):
    return d.sec + d.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--expect-hz", type=float, default=None,
                    help="声明频率，用于判定『实测 vs 声明』")
    ap.add_argument("--target-frame", default="base_footprint",
                    help="TF 检查的基准帧（默认 base_footprint）")
    ap.add_argument("--window", type=float, default=3.0, help="采样窗口时长（秒）")
    ap.add_argument("--timeout", type=float, default=15.0, help="整体超时（秒）")
    ap.add_argument("--ignore-clock", action="store_true",
                    help="跳过时钟源/延迟检查（用 ros2 topic pub 造数据时必需：它不填 stamp）")
    args = ap.parse_args()

    rclpy.init()
    # 体检工具本身必须用**墙钟**：driver 用 sim 时间时才看得出"时钟源不一致"。
    node = Node("driver_check", parameter_overrides=[Parameter("use_sim_time", value=False)])
    deadline = time.monotonic() + args.timeout
    print(f"=== 体检 {args.topic}（窗口 {args.window}s，基准帧 {args.target_frame}）===")

    # ── 1. 发布者与类型 ────────────────────────────────────────────────────
    # ⚠️ 图的发现是**异步**的：进程刚起来时这里查不到任何发布者 ⇒ 不等就会误报"没有发布者" ✗。
    infos = []
    while time.monotonic() < deadline and not infos:
        infos = node.get_publishers_info_by_topic(args.topic)
        if not infos:
            rclpy.spin_once(node, timeout_sec=0.2)
    if not infos:
        report(BAD, "发布者", "没有发布者（话题名拼错 / 驱动没起来 / 域或 RMW 不一致）")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    types = sorted({i.topic_type for i in infos})
    names = sorted({f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in infos})
    report(OK, "发布者", f"{len(infos)} 个 · {', '.join(names)}")
    if len(types) > 1:
        report(BAD, "类型冲突", f"同一话题有多个类型：{types}")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    type_name = types[0]
    report(OK, "类型", type_name)

    # ── 2. QoS：能不能收到，取决于两侧是否兼容 ─────────────────────────────
    # 判据（发布者 ⇒ 订阅者）：reliable 能收 best_effort 的；反过来不行。
    # durability：volatile 订阅者收不到 transient_local 之外的"历史"，反过来 transient_local 收得到 volatile。
    q = infos[0].qos_profile
    rel = {ReliabilityPolicy.RELIABLE: "reliable", ReliabilityPolicy.BEST_EFFORT: "best_effort"}.get(
        q.reliability, "—")
    dur = {DurabilityPolicy.VOLATILE: "volatile", DurabilityPolicy.TRANSIENT_LOCAL: "transient_local"}.get(
        q.durability, "—")
    # ⚠️ rclpy 的 publisher info 里 history/depth 常常是 UNKNOWN ⇒ 别把它猜成 keep_all 报出去 ✗。
    hist = {HistoryPolicy.KEEP_LAST: f"keep_last({q.depth})", HistoryPolicy.KEEP_ALL: "keep_all"}.get(
        q.history, "—")
    report(OK, "QoS", f"reliability={rel} durability={dur} history={hist}")
    if rel == "best_effort":
        report(WARN, "QoS 兼容性",
               "发布者是 best_effort：**reliable 订阅者收不到它** ⇒ 订阅侧必须放宽到 best_effort")
    if dur == "transient_local":
        report(WARN, "QoS 兼容性",
               "发布者是 transient_local：volatile 订阅者**拿不到它已发布的历史**，只能收之后的数据")

    # 用发布者的 QoS 订阅：否则我们自己就收不到，后面几项没法测。
    msg_type = get_message(type_name)
    # ⚠️ 我们按**最弱**配（best_effort + volatile）⇒ 任何发布者都能送数据过来 ✓。
    #    目的是"能收到并测量"，不是为了复现兼容性错误（那个由上面几行的报告负责）。
    sub_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    samples = []
    node.create_subscription(msg_type, args.topic, lambda m: samples.append((time.monotonic(), m)), sub_qos)

    # ── 3/4. 频率、抖动、时间戳 ────────────────────────────────────────────
    deadline = time.monotonic() + args.window + 6.0   # 按阶段重新计时，别让上一阶段吃掉
    while time.monotonic() < deadline and len(samples) < 200:
        rclpy.spin_once(node, timeout_sec=0.1)
        if samples and time.monotonic() - samples[0][0] >= args.window:
            break
    if not samples:
        report(BAD, "数据", f"{args.window}s 内没收到任何消息（QoS 不兼容 / 驱动没发 / 被过滤）")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    stamps = [t for t, _ in samples]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])] or [0.0]
    hz = len(samples) / (stamps[-1] - stamps[0]) if stamps[-1] > stamps[0] else 0.0
    jitter = statistics.pstdev(gaps) * 1000 if len(gaps) > 1 else 0.0
    report(OK, "频率", f"实测 {hz:.1f} Hz · 抖动(σ) {jitter:.1f} ms · 采样 {len(samples)} 条")
    if len(infos) > 1:
        report(WARN, "频率读法", f"该话题有 {len(infos)} 个发布者 ⇒ 上面这个数是**它们的总和**，"
                                 "不是单个驱动的频率")
    if args.expect_hz:
        off = abs(hz - args.expect_hz) / args.expect_hz
        level = OK if off <= 0.3 else (WARN if off <= 0.5 else BAD)
        report(level, "频率对齐", f"实测 {hz:.1f} vs 声明 {args.expect_hz:.1f}（偏差 {off*100:.0f}%）")

    # 发现稳定后再报一次发布者名字：刚起来时 rclpy 常给 `_NODE_NAME_UNKNOWN_`（排障时会误导）。
    settled_infos = node.get_publishers_info_by_topic(args.topic)
    settled = sorted({f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in settled_infos})
    if settled and settled != names:
        report(OK, "发布节点（发现稳定后）", ", ".join(settled))
    # 同一话题上多个发布者、QoS 不同时：订阅侧必须按**最弱**那个配，否则只能收到一部分 ✗。
    # 典型：/joint_states 上一个 transient_local、一个 volatile ⇒ 订阅侧必须 volatile。
    if len(settled_infos) > 1 and any(
            i.qos_profile.reliability != settled_infos[0].qos_profile.reliability or
            i.qos_profile.durability != settled_infos[0].qos_profile.durability
            for i in settled_infos):
        report(WARN, "多发布者 QoS 不同",
               f"{len(settled_infos)} 个发布者的 QoS 不一致 ⇒ 订阅侧要按**最弱**的配"
               "（volatile + 合适的 reliability），否则只能收到其中一部分 ✗")

    # 系统是不是在跑 sim 时间？/clock 有发布者就说明"整个系统用仿真时钟" ⇒
    # 那么本工具（按墙钟比）差出 1.7e9 s 是**正常现象**，不是驱动的问题 ✓。
    sim_time = bool(node.get_publishers_info_by_topic("/clock"))
    # 话题类型是**运行时**从发布者拿的 ⇒ 不能假定消息有 header（`std_msgs/String` 之类就没有）；
    # 但只要有 header，`stamp`/`frame_id` 必定存在（`std_msgs/Header` 的字段）⇒ 只探测这一层。
    header = getattr(samples[-1][1], "header", None)
    stamp = header.stamp if header is not None else None
    frame_id = header.frame_id if header is not None else None

    if args.ignore_clock:
        # ⚠️ `ros2 topic pub` 造的数据**不填 stamp**（实测 sec=0）⇒ 这两项没有意义。
        report(WARN, "时钟源", "按 --ignore-clock 跳过（CLI 造的数据不带时间戳）")
        report(WARN, "端到端延迟", "跳过（同上）")
    elif stamp is None:
        report(WARN, "时间戳", "消息没有 header.stamp ⇒ 判不了时钟源，也只能靠到达时刻估延迟")
    else:
        now_s = node.get_clock().now().nanoseconds * 1e-9
        age = now_s - duration_to_sec(stamp)
        if abs(age) <= 1.0:
            report(OK, "时钟源", f"与本机墙钟差 {age * 1000:+.0f} ms（同一时钟源）")
            report(OK, "端到端延迟", f"最新一条 {age * 1000:+.0f} ms")
        elif sim_time:
            report(WARN, "时钟源",
                   "系统在跑**仿真时钟**（/clock 有发布者）⇒ 与墙钟差一个数量级是正常的 ✓；"
                   "但**消费节点必须 use_sim_time=true**，否则 TF 查询与超时判定全错 ✗")
            report(WARN, "端到端延迟", "跳过（时钟源不同，墙钟差值没有意义）")
        else:
            report(BAD, "时钟源",
                   f"时间戳与本机差 {age:+.1f} s，且**没有** /clock 发布者 ⇒ 驱动与消费方时钟源不一致"
                   "（一方 sim 时间、一方墙钟）✗")
            report(WARN, "端到端延迟", "跳过（时钟源不同）")
    # ── 5. frame_id 在不在 TF 树里 ────────────────────────────────────────
    if not frame_id:
        report(WARN, "TF", "消息没带 frame_id ⇒ 跳过")
    else:
        buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(buffer, node)
        found = False
        tf_deadline = time.monotonic() + 6.0
        while time.monotonic() < tf_deadline and not found:
            rclpy.spin_once(node, timeout_sec=0.1)
            found = buffer.can_transform(args.target_frame, frame_id, Time())
        if found:
            report(OK, "TF", f"{args.target_frame} → {frame_id} 可查（消费方能投影数据）")
        else:
            report(BAD, "TF", f"{args.target_frame} → {frame_id} 查不到"
                             "（frame_id 拼错 / 该帧没人发到 /tf，或是静态帧没进 /tf_static）")

    bad = results.count(BAD)
    warn = results.count(WARN)
    print(f"--- 结论：✗ {bad} · ⚠️ {warn} · ✓ {results.count(OK)} ---")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
