#!/usr/bin/env bash
# Gazebo headless 冒烟：起仿真栈，验证驱动链真的通了，然后干净收摊。
# 只做**结构断言**（控制器 active / 里程计前进 / 桥有数据），**不判精度** ——
# 仿真精度会抖（姿态误差 0.09~0.21 rad），拿它当门禁必然假红。
# 为什么不进 `colcon test`（要 ~1 GB 内存 + GL 上下文）见 gazebo/README ①；
# 它在 CI 里是独立 job `sim-smoke`，本脚本就是那个 job 的唯一入口。
#
#   ./scripts/run_sim_smoke.sh
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$WS/log/sim-smoke-$STAMP.log"
mkdir -p "$WS/log"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# 固定域 + 限本机：runner 上 DDS 组播不稳，这样才可复现。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1

fails=()
ok()   { printf '  ✓ %s\n' "$*"; }
fail() { printf '  ✗ %s\n' "$*"; fails+=("$*"); }
note() { printf '  %s\n' "$*"; }

setsid ros2 launch robot_sdk_gazebo gazebo.launch.py headless:=true >"$LOG" 2>&1 &
PGID=$!
cleanup() {
  kill -TERM -"$PGID" 2>/dev/null || true
  sleep 2
  kill -KILL -"$PGID" 2>/dev/null || true
}
trap cleanup EXIT
note "launch 进程组 $PGID，日志 $LOG"

# `--once` 偶尔先吐一条 "A message was lost!!!"（它自己抢跑丢帧）⇒ 按"取到数字"判，否则随机假红。
sample_odom_x() {
  local v
  for _ in 1 2 3 4; do
    v=$(timeout 8 ros2 topic echo --once --field pose.pose.position.x /odom 2>/dev/null |
        grep -m1 -E '^-?[0-9]' || true)
    if [ -n "$v" ]; then echo "$v"; return 0; fi
    sleep 1
  done
  return 1
}

# ── 1. 控制器都得到 active ────────────────────────────────────────────────────
# 名单**从 ros2_control.yaml 读**（同一份真相，不再在这里维护第二份副本；
# 跨文件的绑定由 scripts/check_sim_consistency.py 兜底）。
# 逐个点名而不是数个数：README 记过"看着活着、实则半配置"（控制器加载了但没激活）。
controllers=$(python3 - "$WS/src/robot_description/config/ros2_control.yaml" <<'PY'
import sys
import yaml

cm = yaml.safe_load(open(sys.argv[1]))["controller_manager"]["ros__parameters"]
print(" ".join(sorted(k for k, v in cm.items() if isinstance(v, dict) and "type" in v)))
PY
)
note "等控制器 active（最多 90 s）：$controllers"
list=""
for _ in $(seq 1 90); do
  list="$(ros2 control list_controllers 2>/dev/null || true)"
  missing=no
  for c in $controllers; do
    if ! echo "$list" | grep -q "^$c .*active"; then missing=yes; break; fi
  done
  [ "$missing" = no ] && break
  sleep 1
done
for c in $controllers; do
  if echo "$list" | grep -q "^$c .*active"; then
    ok "$c active"
  else
    fail "$c 90 s 内没到 active"
  fi
done

# ── 2+3. 控制器没起来就跳过（那时 /odom 没有发布者，采样只能干等到超时）──
if ! echo "$list" | grep -q "^diff_drive_controller .*active"; then
  note "底盘控制器没起来 ⇒ 跳过 /odom 与 TF 检查"
else
  # 发一条速度，看里程计是否真的跟着物理走（只看增量，绝对值无意义）。
  x0=$(sample_odom_x || true)
  if [ -z "$x0" ]; then
    fail "/odom 采不到样本（控制器在发吗？）"
  else
    timeout 5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
      "{header: {frame_id: 'base_footprint'}, twist: {linear: {x: 0.2}}}" >/dev/null 2>&1 &
    PUB=$!
    sleep 3
    x1=$(sample_odom_x || true)
    wait "$PUB" 2>/dev/null || true
    if [ -z "$x1" ]; then
      fail "发速度后 /odom 采不到样本"
    else
      delta=$(awk -v a="$x0" -v b="$x1" 'BEGIN{printf "%.3f", b-a}')
      note "x: $x0 -> $x1（Δ=$delta，期望 ≥ 0.1：0.2 m/s × ~3 s）"
      if awk -v d="$delta" 'BEGIN{exit !(d >= 0.1)}'; then
        ok "底盘真的动了（里程计跟着物理走）"
      else
        fail "发了 3 s 速度但 /odom 没前进（Δ=$delta）"
      fi
    fi
  fi

  # ⚠️ 别写成 `timeout … tf2_echo … | grep -q`：grep 命中即退出 ⇒ 上游吃 SIGPIPE
  #    ⇒ 配 `pipefail` 会把**成功判成失败**。先落变量再判。
  tf_out=$(timeout 12 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -40 || true)
  if echo "$tf_out" | grep -q "Translation"; then
    ok "TF odom → base_footprint 可查"
  else
    fail "TF odom → base_footprint 查不到（Nav2 要这条链）"
  fi
fi

# ── 4. 桥通了没：用 /imu 判（纯物理，不需 GL）；/scan 是渲染式的，无 GPU 的 runner 上会假红 ──
pub_count=$(ros2 topic info /imu 2>/dev/null | sed -n 's/^Publisher count: *//p' | tr -dc '0-9')
if [ "${pub_count:-0}" -ge 1 ] 2>/dev/null; then
  ok "/imu 有发布者（gz → ROS 的桥通了）"
else
  fail "/imu 没有发布者（桥可能没起来）"
fi

echo
if [ "${#fails[@]}" -eq 0 ]; then
  echo "仿真冒烟：全部通过 ✓"
  exit 0
fi
echo "仿真冒烟：${#fails[@]} 项失败 ✗"
printf '  - %s\n' "${fails[@]}"
echo "launch 日志尾部（$LOG）："
tail -25 "$LOG" | sed 's/^/    /'
exit 1
