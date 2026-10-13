# 一个句柄的引用环 —— 从"取消偶发抛错"到 LSan 归零

> **这份文档回答一个问题**：`ActionHandle` 为什么会**活着不放**，后果是什么，以及怎么用工具把它钉成数字。
>
> - 想先看这个类的设计 → [`sdk-call-flow.md`](sdk-call-flow.md) §7.3
> - 想看它怎么被 Agent 用起来 → [`agent-integration-plan.md`](agent-integration-plan.md)
> - **本文**：一个真实的**生命周期 bug** —— 现象 → 根因 → 修法 → **A/B 实测证据**
>
> 纪律照旧：**【量】** = 本机跑过、有数字；**【推】** = 从框架语义推导。

---

## 0. 一句话

`ActionHandle` 与 rclcpp_action 的 `GoalHandle` **互持**（反馈回调按值捕获了 handle，而 handle 又持有 GoalHandle）⇒ **句柄永不析构**。两个后果：

1. **行为**：`cancelAll()` 拿到一个"早已结束的目标"去取消 ⇒ client 抛 `Goal handle is not known to this client`；
2. **进程内（看不见）**：**每完成一个动作目标就漏一份**（handle + goal handle）。

修法是 7 行的一处改动。实测：**LSan 从 22 处分配 / 1624 字节 → 0**；**TSan 四个示例 0 数据竞争**。

---

## 1. 现象

**症状 A（行为）**：调 `cancelAll()` 会抛 `Goal handle is not known to this client`（Python 侧是 `ValueError`，因为 rclcpp_action 抛的是 `std::invalid_argument`）。规律是**只在那之前有过"已完成的目标"时才出现**。

**症状 B（看不见）**：每完成一个目标泄漏一份——没有工具时只能靠读代码推出来。

---

## 2. 根因：一个环

三段代码咬在一起（都在本仓库/ROS 里，可逐行核对）：

| # | 位置 | 做了什么 |
|---|---|---|
| ① | `navigation_api.cpp:58`、`motion_api.cpp:65`、`motion_api.cpp:130` | `options.feedback_callback = [handle](...){...}` —— **按值捕获 `shared_ptr<ActionHandle>`** |
| ② | `rclcpp_action/client.hpp:458-459` | 建 GoalHandle 时把 `options.feedback_callback` **拷进** GoalHandle；落成成员 `client_goal_handle.hpp:162` |
| ③ | `action_handle.hpp:135` | `goal_future_` 是 `shared_future<GoalHandle::SharedPtr>` —— **handle 持有 GoalHandle** |

于是：

```
ActionHandle ──(③ goal_future_)──► GoalHandle ──(② feedback_callback_)──► lambda ──(① 捕获)──► ActionHandle
```

**引用计数永远 ≥ 1，谁都不放手。**

**为什么这会"爆"成异常**：rclcpp_action 在**结果到达**时就把它从自己的登记表里删掉（`client.hpp:793-794` 的 `goal_handles_.erase(...)`）；而 SDK 的登记册 `active_handles_` 存的是 **weak_ptr**，它**指望 handle 自己死掉后自然失效**。环让 handle 不死 ⇒ 登记里永远躺着一条"僵尸" ⇒ `cancelAll()` 遍历时 `weak.lock()` **成功** ⇒ 拿着它去 `async_cancel_goal` ⇒ client 查表查不到 ⇒ 抛。

> 一句话：**ROS 那边的登记是 weak 且"结果到手即清"；SDK 这边的"清"从来没发生过。**

---

## 3. 修法（7 行，一处）

```cpp
auto result_future = client->async_get_result(goal_handle);
const bool got_result = result_future.wait_until(deadline) == std::future_status::ready;
// 结果一旦请求，这个目标就必然走向终态：放掉对 rclcpp_action goal handle 的引用。
goal_future_ = {};
if (!got_result) {
  client->async_cancel_goal(goal_handle);
  return Error::Timeout("timed out waiting for the action result");
}
return result_future.get();
```

**为什么这一行就够**：放掉 `goal_future_` ⇒ GoalHandle 没有任何强引用了（client 的登记是 weak）⇒ 它析构 ⇒ 它内部的回调跟着析构 ⇒ handle 被释放 ⇒ `active_handles_` 里那条 weak 自然失效 ⇒ `cancelAll` 根本不会碰它。**语义上等价于 ROS 的"结果到手就 erase 自己的登记"。**

⚠️ 注意**两条出口都要放**：正常拿到结果、以及**请求过结果但超时**（`wait()` 超时后会主动取消，那时结果已经请求过、client 迟早会 erase ⇒ 不放就还是会抛）。这条是实测踩出来的，不是想出来的。

---

## 4. 证据 A：行为对照（三种取消场景）【量】

| 场景 | 改前 | 改后 |
|---|---|---|
| 从不调 `wait()`，目标自己跑完 → `cancelAll` | 不抛 | 不抛 |
| `wait()` 正常完成（句柄仍持有）→ `cancelAll` | **抛** | 不抛 |
| `wait()` **超时**后 → `cancelAll` | **抛** | 不抛 |

判据是工具返回里的 `failures` 列表：改前有 `["motion: Goal handle is not known to this client."]`，改后为空。

---

## 5. 证据 B：LSan 前后对照【量】

**做法**：只给**客户端侧**（示例程序）加插桩 —— 泄漏在客户端，mock 不需要。

```bash
colcon build --symlink-install --packages-select robot_sdk_examples \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -g" \
               -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"
# 对着普通 mock 跑（示例发 2 个目标：一个异步带反馈 + 一个阻塞）
install/robot_sdk_examples/lib/robot_sdk_examples/demo_navigation
```

| | 结果 |
|---|---|
| **不带修复** | `SUMMARY: AddressSanitizer: 1624 byte(s) leaked in 22 allocation(s)`；分配栈里明确出现 `rclcpp_action::ClientGoalHandle<...>::ClientGoalHandle(...)` 与 `async_send_goal(...)::{lambda(std::shared_ptr<void>)}::operator()`（就是 `new GoalHandle` 那处），**两处同型**对应两个目标 |
| **带修复** | **0**（LSan 没有任何输出） |
| 功能 | 两次都 `success=1  末距 0.046 m` —— 行为完全不变 |

> 这条对照是关键：**它把"我推出来有个环"变成了"工具数出来的 22 处 → 0"**。

**一个读报告的技巧**：这 16 条**全是 `Indirect leak`、一条 `Direct` 都没有**。LSan 是按**可达性**判的 ——
`Direct` = 没有任何根（全局/栈/寄存器）指向它；`Indirect` = **只从另一个泄漏块可达**。
**全是 indirect ⇒ 泄漏集是一个闭环**（没有任何根通往它们）—— 这正是**引用环**的指纹，
所以看到这个词就该怀疑"是不是互相持有"。

原始报告：[`evidence/asan-leak-before.log`](evidence/asan-leak-before.log)（裁剪版：保留每条头部 + 分配点/我方帧，去掉 asan/libc 内部帧与 BuildId）、[`evidence/asan-leak-after.log`](evidence/asan-leak-after.log)（全文）。
索引与获取方式见 [`evidence/README.md`](evidence/README.md)。

---

## 6. 证据 C：TSan（并发）【量】

⚠️ **Ubuntu 24.04 上 TSan 起不来**：内核默认 `vm.mmap_rnd_bits=32`，TSan 直接
`FATAL: ThreadSanitizer: unexpected memory mapping`（ASan 不受影响，所以上面那步没事）。
**不用改系统参数** —— 只对这个进程关掉 ASLR 即可：

```bash
setarch $(uname -m) -R install/robot_sdk_examples/lib/robot_sdk_examples/demo_navigation
```

四个示例在 TSan 下：**0 条 `data race`**、退出码 0。

| 示例 | 压什么并发路径 |
|---|---|
| `hello_robot` | 状态推送回调 + 感知「最新值」缓存 + 阻塞等待 |
| `demo_navigation` | 异步反馈回调（ROS 线程）+ 主线程阻塞调用 + `spin()` 线程 |
| `demo_motion` | 关节动作 + 夹爪（同一 API 的两条 action） |
| `demo_manipulation` | 具名位姿 → 夹爪 → home（多个目标串起来） |

⚠️ **局限（别把它读成"整个系统无竞争"）**：这是**部分插桩** —— 只有我们的代码带 TSan，`rclcpp` / `rclcpp_action` / DDS 都没带。**只在未插桩那侧发生的数据竞争看不见。** 结论只能说到"**我们代码里的访问没有被 TSan 抓出竞争**"。

**报告长什么样 / 怎么确认工具真的在工作**：拿一个**故意造竞争**的小程序当阳性对照
（[`evidence/tsan-positive-control.cpp`](evidence/tsan-positive-control.cpp)），它的报告是：

```
WARNING: ThreadSanitizer: data race (pid=…)
  Read of size 4 at 0x… by thread T2:
    #0 bump_racy() …/tsan_demo.cpp:9
  Previous write of size 4 at 0x… by thread T1:
    #0 bump_racy() …/tsan_demo.cpp:9
  Location is global 'race_var' of size 4 at 0x…
ThreadSanitizer: reported 1 warnings
```

三段：**两侧各自的调用栈** + `Location is global 'x'`（有时还给 happens-before 链）+ 末尾汇总；
加锁修好后**完全安静**。⚠️ 另一个要点：**报出竞争 ≠ 这次跑出了错值** —— 那个对照程序的计数器恰好没丢更新，
TSan 照样报。这就是"用工具"而不是"跑一遍看数字对不对"的理由。

⚠️ 两个补充：① ASLR 那个 FATAL 是**概率性**的（小程序有时不撞），所以 `setarch -R` 是**可靠做法**而非每次必需；
② **CMake 会缓存 sanitizer 参数**，想回到干净构建必须 `rm -rf build/<pkg>`。

原始报告：[`evidence/tsan-demos.log`](evidence/tsan-demos.log)（四个示例全文，每份 sanitizer 行数 0）、[`evidence/tsan-aslr-fatal.log`](evidence/tsan-aslr-fatal.log)、[`evidence/tsan-positive-control.log`](evidence/tsan-positive-control.log)。

---

## 7. 教训

1. **weak 登记 + 拥有型回调是危险组合**：登记册指望对象自己死，而回调让对象不死。两者分开看都合理，合起来就是环。
2. **契约要在两侧都兑现**：同一个 `any_orientation` 标志，**规划侧兑现了、上报侧没兑现**（见 §8）—— 把同一语义拆到两条路径上处理，是这类漂移的温床。
3. **顺序别反**：先拿到**能稳定复现的行为失败**，再上内存/并发工具把它钉成数字。反过来会在工具的噪声里迷路。

---

## 8. 同一个类上的第二个契约 bug（顺带修掉）

`MotionAPI` 的头注释与 `MoveToPose.action` 的字段注释都写着"`any_orientation` 为真时 `final_orientation_error` **恒为 0**（不是没测，是没要求）"，但 `moveit_adapter` **无条件**拿"实际末端姿态"与 goal 里的姿态比 —— 而 `any_orientation=true` 时那个姿态是客户随手填的（SDK 默认**单位四元数**），于是报出 2.77 / 1.926 rad。修法：把 `goal->any_orientation` 存下来，只在**客户真要求姿态时**才算该字段。修后实测：`any_orientation=true` ⇒ `0.0`；`=false` ⇒ 仍是实测量（真栈探针两个 seed 0.036 / 0.036 rad）。

---

## 9. 复现清单

```bash
source /opt/ros/jazzy/setup.bash && cd <repo>/ros2_ws && source install/setup.bash
ros2 run robot_sdk_sim mock_robot_node &          # 普通构建即可

# LSan：把 §3 那行注释掉就是"不带修复"，恢复就是"带修复"
colcon build --symlink-install --packages-select robot_sdk_examples \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=address -g" -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address
install/robot_sdk_examples/lib/robot_sdk_examples/demo_navigation

# TSan（Ubuntu 24.04 必须加 setarch -R）
colcon build --symlink-install --packages-select robot_sdk_core robot_sdk_examples \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread
setarch $(uname -m) -R install/robot_sdk_examples/lib/robot_sdk_examples/demo_navigation

# ⚠️ 收尾：CMake 会把 sanitizer 参数**缓存**住，不带参数重建是清不掉的 —— 必须删 build 目录
rm -rf build/robot_sdk_core build/robot_sdk_examples && colcon build --symlink-install
```
