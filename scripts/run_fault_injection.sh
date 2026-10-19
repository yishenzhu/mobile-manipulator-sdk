#!/usr/bin/env bash
# 故障注入：把驱动接入最常见的四类**静默错误**造出来，看体检脚本能不能抓到。
#
#   ./scripts/run_fault_injection.sh
#
# 为什么值得做成脚本：这四类错误的共同点是"**不报错、只是没数据或数据不对**"，
# 所以"可观测性"只能用"故意做坏 ⇒ 必须报警"来证明。
#
# 期望（每类只应点亮**一项**）：
#   ① QoS 错   ⇒ ⚠️ 发布者是 best_effort（reliable 订阅者收不到）
#   ② 帧名错   ⇒ ✗ TF base_footprint → no_such_frame 查不到
#   ③ 频率掉   ⇒ ✗ 频率对齐（实测 ~2 Hz vs 声明 10 Hz）
#   ④ 时间戳错 ⇒ ✗ 时钟源（差 ~56 年，且没有 /clock）
set -o pipefail
source /opt/ros/jazzy/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1

TOPIC=/probe/scan
TYPE=sensor_msgs/msg/LaserScan
OUT="$WS/log/fault-injection-$(date +%Y%m%d-%H%M%S).log"
mkdir -p "$WS/log"
: > "$OUT"
TF_PID=""
PUB_PID=""
cleanup() {
  [ -n "$PUB_PID" ] && kill "$PUB_PID" 2>/dev/null
  [ -n "$TF_PID" ] && kill "$TF_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

# 造一棵最小 TF 树：base_footprint → laser_link。
# 否则"帧名"那项在**每个** case 里都会红（没有 TF 树谁都查不到）⇒ 证据就混了。
# ⚠️ Jazzy 的这个工具**必须**用 --frame-id/--child-frame-id（位置参数会被拒绝，实测 exit 1）
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_footprint --child-frame-id laser_link >/dev/null 2>&1 &
TF_PID=$!
sleep 1

case_run() {  # $1=标题 $2=速率 $3=体检脚本额外参数 $4...=消息体
  local title=$1 rate=$2 extra=$3
  shift 3
  ros2 topic pub -r "$rate" --qos-reliability best_effort "$TOPIC" "$TYPE" "$@" >/dev/null 2>&1 &
  PUB_PID=$!
  sleep 1
  echo "=== $title ===" | tee -a "$OUT"
  # shellcheck disable=SC2086
  timeout 40 python3 "$WS/scripts/check_driver_topic.py" "$TOPIC" --window 4 --expect-hz 10 $extra \
    2>&1 | tee -a "$OUT" || true
  echo | tee -a "$OUT"
  kill "$PUB_PID" 2>/dev/null || true
  wait "$PUB_PID" 2>/dev/null || true
  PUB_PID=""
}

# ①②③ 用 --ignore-clock：`ros2 topic pub` 不填 stamp（实测 sec=0），否则"时钟源"会跟着红。
case_run "① QoS：best_effort 发布者（声明 10 Hz）" 10 --ignore-clock \
  "{header: {frame_id: 'laser_link'}}"
case_run "② 帧名：frame_id 不存在（10 Hz）" 10 --ignore-clock \
  "{header: {frame_id: 'no_such_frame'}}"
case_run "③ 频率：2 Hz（声明 10 Hz）" 2 --ignore-clock \
  "{header: {frame_id: 'laser_link'}}"
# ④ 目标就是时钟源 ⇒ 不加 --ignore-clock
case_run "④ 时间戳：固定在 1970（10 Hz）" 10 "" \
  "{header: {frame_id: 'laser_link', stamp: {sec: 0, nanosec: 0}}}"

echo "留档：$OUT"
