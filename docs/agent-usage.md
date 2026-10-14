# 用一句话驱动机器人

`robot_operator` 这个 agent（工具面收窄到 `mcp_robot_*` + 低温度）用 `walle-cli --agent robot_operator`
选中；不传就退回 `default`。下面是实际跑法与踩过的坑。

## 怎么跑

```bash
# 终端 1：起机器人（mock 或真栈都行 —— "换后端不改 agent"就体现在这里）
export ROS_DOMAIN_ID=42        # ⚠️ 必须与 ~/walle/.agent/mcp.yaml 给 MCP 子进程设的域一致
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run robot_sdk_sim mock_robot_node --ros-args \
  -p robot_description_file:=$(ros2 pkg prefix robot_description)/share/robot_description/urdf/mobile_manipulator.urdf

# 终端 2：服务端（常驻）+ 客户端（说人话）
cd ~/walle
./scripts/run.sh --no-obs
PYTHONPATH=.. python -m walle.channel.cli --agent robot_operator
```

## 实测（CLI，`--agent robot_operator`）

```
You> 读一下机器人现在的状态，用一句话告诉我它的电量和工作状态
🔧 read {'path': '.../robot-operating/SKILL.md'}      ← 技能渐进披露 ⇒ agent 必须能用 read
🔧 mcp_robot_get_status {'fresh': True}
✅ {...}
→ "机器人当前电量约 97.8%、未在充电，处于 IDLE（空闲），无急停、错误码 0。"
```

怎么确认选上的就是它 —— 服务端日志那一行会写出 agent 名：

```
cli client connected: cli-4a852c892ce2 agent=robot_operator
```

再反过来问一句"用 `bash` 工具执行 echo hi"：它会**拒绝并说明自己没有这个工具**，只列出 `read`
与 `mcp_robot_*` —— 工具面确实收窄了。

## 两个坑

1. **DDS 域必须一致**：`.agent/mcp.yaml` 给 MCP 子进程设了 `ROS_DOMAIN_ID=42`，机器人若起在默认域，
   agent 会（**正确地**）报"话题上没有发布端 / 服务不可用"—— 看着像 SDK 坏了，其实只是没连上。
2. **运动类命令会弹审批**：`conf.yaml` 的规则先跑，没被规则挡住的运动工具会问人（敲 y/n）。
   这是**该有的样子**（危险动作人工确认）；无人值守的场景需要另配审批策略。

## 这次验证到 / 暴露出的

1. **上一轮的错误面工作直接兑现**：域不一致那次，agent 拿到的正是
   `UNAVAILABLE: topic '/joint_states' has no publisher at all`，于是它**正确区分了"服务端不在"与"忙"**
   （技能里写着"忙"的特征是消息带 `rejected`），并**拒绝编造数据、拒绝盲发运动指令**。
   —— 错误面的信息量，直接决定 agent 能不能自己判断。
2. **一个能力缺口**：工具只有 `action=open|close`，而 **SDK 没有暴露关节限位** ⇒
   "把夹爪张开到**最大**"这类目标 agent 无从判断（实测时它老实说"没法用读数证实"）。
   可行的补法：SDK 侧加一个"描述限位"的工具（MoveIt 后端有 `get_joint_limits` 类服务可转发）。

**没做**：eval 黄金任务集（量化"工具选择 / 参数 / 有没有盲发"，属之前跳过的可观测+eval 那摊）。
