#!/usr/bin/env bash
# 跑测试并把结果**按时间戳留档**，再把失败的测试名直接打出来（不用翻 XML）。
# 不这么做的话，一轮通过的重跑会覆盖掉上一轮失败的证据 —— robot_sdk_tests 有已知的时序 flake，吃过这个亏。
# ⚠️ 只用 -e pipefail：ROS 的 setup.bash 会引用未定义变量，加 -u 会当场崩（实测）。
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$WS/log/test-results/$STAMP"
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# 先清上一轮结果：改名/删除过的测试会留下陈旧 XML，把本轮数字和留档都污染（实测踩过，虚增 4 项）
rm -rf "$WS"/build/*/test_results "$WS"/build/*/pytest.xml
# rc 先接住，否则 -e 会让 colcon test-result 都跑不到
colcon test "$@" && rc=0 || rc=$?
colcon test-result --all || true

# 留档：每份结果 XML（带包名前缀）+ colcon 的 latest_test
find "$WS/build" -name "*.xml" \( -path "*test_results*" -o -name "pytest.xml" \) 2>/dev/null |
  while read -r f; do
    pkg="$(echo "$f" | sed -E "s|^$WS/build/([^/]+)/.*|\1|")"
    cp "$f" "$OUT/${pkg}-$(basename "$f")" 2>/dev/null || true
  done

python3 - "$OUT" <<'PY'
import glob
import sys
import xml.etree.ElementTree as ET

bad = []
for path in sorted(glob.glob(sys.argv[1] + "/*.xml")):
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        continue
    for case in root.iter("testcase"):
        node = case.find("failure") if case.find("failure") is not None else case.find("error")
        if node is not None:
            first = (node.get("message") or "").splitlines()[0][:160]
            bad.append((case.get("classname", ""), case.get("name", ""), first))

for cls, name, msg in bad:
    print(f"  FAILED  {cls}::{name}")
    print(f"          {msg}")
print(f"  结果留档: {sys.argv[1]}")
print("  全部通过" if not bad else f"  {len(bad)} 个失败（已按时间戳留档）")
PY

exit $rc
