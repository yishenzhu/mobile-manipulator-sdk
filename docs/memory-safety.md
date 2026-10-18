# 内存安全与 sanitizer

这个 SDK 横跨 **C++ ↔ Python**（pybind11）、又有**多线程 executor + 回调**，最容易出、也最难在现场定位的是
**生命周期**问题：悬垂引用、双重释放、回调持有已析构对象、数据竞争。它们几乎都是"静默"的 ——
不报错、只是行为诡异，等流到使用者手上基本没法查。

所以这里的做法是：**把这类问题变成 CI 里能被自动抓到的东西**，而不是靠 review 的运气。

---

## 怎么跑

```bash
# 1) 构建到**独立目录**（不动日常的 build/ install/）
./scripts/build_sanitizers.sh address,undefined   # ASan + UBSan（默认）
./scripts/build_sanitizers.sh thread              # TSan

# 2) 跑核心测试，sanitizer 报出来的问题会原样打出来
./scripts/run_sanitizer_tests.sh asan
./scripts/run_sanitizer_tests.sh tsan
```

产物目录按 sanitizer **分开**：`build-san-address-undefined/`、`build-san-thread/`（`.gitignore` 里的 `/build-*`、`/install-*` 已覆盖）。
**ASan 与 TSan 不能共用同一份产物** —— 混在一起只会看到 `FATAL: unexpected memory mapping` 之类的假故障（实测踩过）。

CI 里是独立的 `sanitizers` job（见 `.github/workflows/ci.yml`），与主 job 分开：失败原因完全不同
（内存/竞争 vs 逻辑），混在一起看不出是谁的问题。

---

## 两个环境坑（都实测过）

1. **TSan 在新内核上起不来**：`FATAL: ThreadSanitizer: unexpected memory mapping 0x…`
   原因是 ASLR 位数偏大（`vm.mmap_rnd_bits`），TSan 的地址空间假设被打破。
   **不需要 root**：`setarch -R`（关掉该进程的 ASLR）即可 —— 脚本里已经带上。
2. **ROS 自身的库不是 sanitizer 版本** ⇒ 开 leak 检测会被它内部的大量分配淹没。
   所以脚本默认 `detect_leaks=0`（先保证内存错误/UB 这类**硬**问题没有噪音）。
   真要查我们自己的泄漏，见下面"还没做的"。
3. **Python + ASan 目前跑不通**：`LD_PRELOAD=libasan.so python3 …` 会在
   `asan_interceptors.cpp "real___cxa_throw != 0"` 上 CHECK 失败（`verify_asan_link_order=0` 也绕不过）。
   所以 Python 侧的压力测试暂时在**非 sanitizer**环境下做（见下）。

---

## 现有结论（都要能复现）

- **ASan + UBSan**：核心 16 条 gtest **全部通过、零报告** ✓ —— 其中
  `test_robot_lifecycle` 是专门为这类问题写的：**在 `spin()` 线程运行期间反复构造/析构 `Robot`**，
  把 status/perception/motion/navigation/tf/config 都碰一遍 —— "悬垂回调 / 双重释放"最容易出在这个场景。
- **TSan**：能跑，只在 `test_transform` 上报过竞争，两条已分别定性：
  - **① 是我们的问题**：`TransformManager` 与 `NodeSpinner` 的**析构顺序** —— spinner 必须声明在
    tf **之后**，否则 spin 线程可能正在执行已析构对象的回调（悬垂访问）。**已修**，见 `test/test_transform.cpp`。
  - **② 是库的问题**：报告落在 `libtf2.so` 内部（预编译库的锁对 TSan 不可见）⇒ 按 `scripts/tsan.supp`
    的 `race:memcmp` 抑制。⚠️ 抑制等于**放弃对该点的监控** ✗ —— 所以这一类改用"顺序约定 +
    生命周期压力测试（`test_robot_lifecycle`）"兜底，不靠 suppression 假装干净。

---

## 顺带发现的两个"契约/生命周期"问题（都不是靠 sanitizer，而是靠盯行为）

1. **SDK 不持有调用方传入的回调组** ⇒ 内联传 `node->create_callback_group(...)` 时组会随语句结束析构，
   action 的 goal response 再也回不来（详见 CHANGELOG 与 `docs/contract.md`）。
   根因是 rclcpp 的节点对这类组**只存 `weak_ptr`**（`node_base.hpp`）。
2. **`stop()` 早于 `spin()` 进入时会被吃掉** ⇒ `spin()` 永久阻塞（`stop()` 只是 `executor.cancel()`，
   而 rclcpp 的 `spin()` 会重置该状态）。契约写的是"直到 `stop()` 被调用"，实现没满足。
   已写进 `test_robot_lifecycle.cpp` 的注释；**没有改产品行为**。

---

## 已知限制 / 还没做的

- **覆盖不全**：只编了 `robot_sdk_core`（+ 依赖的消息包）。真栈（`ros2_control` + `move_group`）、
  Nav2 适配器、mock 都没进 sanitizer 构建。
- **Python 侧泄漏**：目标做法是"反复 `Robot()` + `del` + `gc.collect()` ⇒ 断言 ROS 句柄/订阅数不增长"，
  但 Python+ASan 目前起不来（见坑 3）⇒ 暂时只能靠 C++ 侧的生命周期压力测试兜。
- **leak 检测**：要查我们自己的泄漏，需要 LSan suppression 把 ROS/Python 的分配排除掉，
  否则噪音里捞不到信号。
- **TSan 那两条**：需要按"两侧完整栈 + `TransformManager` 析构顺序"继续判归属。
