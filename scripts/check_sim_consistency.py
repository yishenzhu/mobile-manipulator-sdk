#!/usr/bin/env python3
"""仿真侧"同一概念出现在多个文件"的一致性检查（和契约门禁同一个思路）。

为什么需要：这几处描述的是**同一个东西**，但物理上分散在不同文件 —— 改一处忘另一处，
症状是"栈能起来、但某个控制器/关节静默缺失"（`/joint_states` 两套数据就是这么来的）。
所以不去硬合并不同语义的代码，而是把这些副本**绑定**成一份真相：

1. URDF `<ros2_control>` 的关节  ==  ros2_control.yaml 里 joint_state_broadcaster 的关节
2. ros2_control.yaml 声明的控制器  ==  gazebo.launch.py 实际 spawn 的控制器
3. run_sim_smoke.sh 不再硬编码控制器名单 —— 它必须从 ros2_control.yaml 读
"""
import pathlib
import re
import sys

import yaml

WS = pathlib.Path(__file__).resolve().parent.parent
URDF = WS / "src/robot_description/urdf/mobile_manipulator.urdf.xacro"
CFG_YAML = WS / "src/robot_description/config/ros2_control.yaml"
LAUNCH = WS / "src/robot_sdk_gazebo/launch/gazebo.launch.py"
SMOKE = WS / "scripts/run_sim_smoke.sh"

fails = []


def check(name, ok, detail=""):
    print(("  ✓ " if ok else "  ✗ ") + name + ("：" + detail if detail else ""))
    if not ok:
        fails.append(name)


cfg = yaml.safe_load(CFG_YAML.read_text())
cm = cfg["controller_manager"]["ros__parameters"]
declared = sorted(k for k, v in cm.items() if isinstance(v, dict) and "type" in v)

# 1) 关节集合
block = re.search(r"<ros2_control[^>]*>(.*?)</ros2_control>",
                  URDF.read_text(), re.S)
urdf_joints = set(re.findall(r'<joint name="([^"]+)"', block.group(1)))
jsb_joints = set(cfg["joint_state_broadcaster"]["ros__parameters"]["joints"])
check("JSB 的关节 == URDF <ros2_control> 的关节", jsb_joints == urdf_joints,
      "仅 yaml: %s / 仅 URDF: %s" % (sorted(jsb_joints - urdf_joints),
                                     sorted(urdf_joints - jsb_joints)))

# 2) 控制器集合：yaml 声明 vs launch 实际 spawn
spawned = set(re.findall(r'spawner\(\s*"([a-z_]+)"', LAUNCH.read_text()))
check("launch spawn 的控制器 == yaml 声明的控制器",
      spawned == set(declared),
      "仅 launch: %s / 仅 yaml: %s" % (sorted(spawned - set(declared)),
                                       sorted(set(declared) - spawned)))

# 3) 冒烟脚本不得再持有第二份控制器名单
smoke = SMOKE.read_text()
hard = re.search(r"for c in ([a-z_ ]+); do", smoke)
reads_yaml = "ros2_control.yaml" in smoke
check("冒烟脚本从 yaml 读控制器名单（不再硬编码）",
      reads_yaml and hard is None,
      "仍硬编码: %s" % (hard.group(1) if hard else "—"))

print()
if fails:
    print("  ✗ %d 项不一致（同一概念被写成了多份）" % len(fails))
    sys.exit(1)
print("  ✓ 全部一致")
