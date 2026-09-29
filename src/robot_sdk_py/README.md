# robot_sdk_py

Robot SDK 的 **Python 绑定**（pybind11）。

**只有一个实现**：Python 和 C++ 调的是同一份核心（`robot_sdk_core`），所以行为、错误语义
完全一致 —— 不存在"两套实现慢慢漂移"这回事。

## 怎么跑

```bash
ros2 run robot_sdk_sim mock_robot_node      # 终端 1：假机器人

python3 $(ros2 pkg prefix robot_sdk_py)/share/robot_sdk_py/python/hello_robot.py
python3 $(ros2 pkg prefix robot_sdk_py)/share/robot_sdk_py/python/demo_navigation.py
```

## 两条边界约定（有意为之，别改）

**① 异常只在语言边界上抛。**
C++ 侧刻意**不用异常**（实时性 + 让异常不穿越语言边界），失败用 `Result<T>`/`Error` 表达。
到了 Python 这边，失败**抛 `RobotSdkError`**、成功把值投影成 `dict`：

```python
try:
    status = robot.status().wait_for_status(5.0)
    print(status["battery_percentage"])
except robot_sdk.RobotSdkError as error:
    print(error)          # "TIMEOUT: no robot status received within timeout"
```

抛出来的消息里带着**稳定的错误码**（`TIMEOUT` / `INVALID_ARGUMENT` / …），客户据此分支。

**② GIL：阻塞调用放开、回调自己拿。**
- 阻塞式方法（`wait_for_*` / `go_to` / `move_*`）在 C++ 侧**放开 GIL**，否则后台 `spin()`
  线程动不了 ⇒ 死锁。所以 Python 里的惯用法是：后台线程 `robot.spin()` + 主线程调阻塞 API。
- 反过来，**回调（`on_feedback` / `on_status`）来自 ROS 线程、没有 GIL**，绑定里必须自己拿。
  ⚠️ 而且不能把 `py::object` 直接捕获进那个 `std::function` —— C++ 侧（`ActionHandle`）会在
  **没有 GIL 的线程里拷它**，拷贝就是 `inc_ref`，会当场 core dump（实测踩过）。
  绑定里用 `GuardedCallback`（shared_ptr 包住 + 带 GIL 的 deleter）解决。

## 实测到什么程度

```
✅ colcon build：10 packages，0 警告 0 错误
✅ colcon test：绑定自身 4 条用例全过（字段默认值 / 子配置可写 / goal 构造与校验 /
   "失败抛 RobotSdkError"这条约定）
✅ 两个示例对着 mock 机器人真跑通：
     hello_robot      状态 / 激光 360 条 / IMU / 深度图 64x48 / 点云 / 服务查询 / 位姿
     demo_navigation  反馈回调一路到"剩 0.08 m" ⇒ SUCCEEDED ⇒ 位姿 x=1.50 y=0.50 yaw=0.40
                      阻塞调用回原点 ⇒ success=True 末误差 0.041
```

## 没绑的东西

`TransformManager`（`robot.tf()`）**没有**暴露 —— 它的三个方法返回的是 ROS 消息
（`TransformStamped` / `PoseStamped`），要么再写一层投影、要么把消息类型也绑出来。
需要的话说一声，那是一次独立的小改动。
