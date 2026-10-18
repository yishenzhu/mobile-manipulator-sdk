#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <string>

#include "robot_sdk/robot.hpp"

using namespace robot_sdk;

namespace {

class RobotLifecycleTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

/// 生命周期压力：反复构造/析构 `Robot`，并把各子系统都碰一遍。
///
/// 这是最容易被 sanitizer 抓到东西的地方：`Robot` 构造时会建节点、回调组、起 spin
/// 线程、装订阅/客户端/缓存，析构时要把它们按正确顺序拆干净 —— 顺序错一点就是
/// "悬垂回调"（线程还在跑，对象已经没了）或双重释放。
/// **价值主要在 ASan/UBSan 构建下**（见 `scripts/run_asan_tests.sh`）；普通构建下它只保证"能跑完"。
TEST_F(RobotLifecycleTest, ConstructDestroyRepeatedly) {
  for (int i = 0; i < 6; ++i) {
    RobotConfig config;
    config.node_name = "lifecycle_probe_" + std::to_string(i);  // 不同名：避免同名节点互扰

    auto robot = std::make_unique<Robot>(config);

    // 让 spin 线程真的跑起来 —— "线程在跑时各子系统被创建/销毁"才是重点
    std::thread spinner([&robot]() { robot->spin(); });
    // 只为了让 spin 落位（sanitizer 关心的就是"线程正在跑"）。⚠️ 正确性**不再依赖**这句：
    // stop() 早于 spin() 的情况已由 RobotRuntime 自己兜住（见下面两个用例）。
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 走一遍各子系统：订阅、客户端、缓存、回调组都在这一步建出来
    robot->status();
    robot->perception();
    robot->motion();
    robot->navigation();
    robot->tf();
    robot->config();

    robot->stop();      // 先按契约让 spin() 返回
    spinner.join();
    robot.reset();      // 再析构：线程/executor/回调的拆除顺序是重点
  }
}

/// `stop()` 早于 `spin()`：契约写的是"直到 `stop()` 被调用"，所以 `spin()` 必须返回。
///
/// 这是 rclcpp 的**已知竞态**（ros2/rclcpp#3172、#1454）：`stop()` 只是 `executor.cancel()`，
/// 而 `spin()` 入口的 `spinning.exchange(true)` 会**无条件**覆盖那个状态 ⇒ 早到的 cancel 被吃掉、
/// `spin()` 永久阻塞。上游只在 rolling 修了，且改动了类布局 ⇒ jazzy/kilted/lyrical 都拿不到
/// ⇒ SDK 侧自己兜（`RobotRuntime::stop_requested_`，语义与上游修法一致：意图只消费一次）。
///
/// ⚠️ 断言方式：**轮询等待**而不是直接 `join()` —— 万一缺陷回来，这里要"红"，不能把 CI 挂死。
TEST_F(RobotLifecycleTest, StopBeforeSpinIsHonored) {
  RobotConfig config;
  config.node_name = "lifecycle_stop_first";
  auto robot = std::make_unique<Robot>(config);

  robot->stop();  // 先停：此时 spin() 还没进入

  std::atomic<bool> returned{false};
  std::thread spinner([&returned, &robot]() {
    robot->spin();
    returned.store(true);
  });

  for (int i = 0; i < 40 && !returned.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 最多等 2 s
  }

  const bool returned_ok = returned.load();
  EXPECT_TRUE(returned_ok) << "stop() 早于 spin() 被吃掉 ⇒ spin() 永久阻塞（rclcpp#3172）";
  if (returned_ok) {
    spinner.join();
  } else {
    // 失败路径：线程正卡在 executor 里，join 会挂死 ⇒ 放手让它随进程结束；
    // 同理不析构 robot，避免"线程还在跑、对象已销毁"。
    spinner.detach();
    robot.release();
  }
}

/// `stop()` 的意图**只消费一次**：`stop()` 先到 ⇒ 那一次 `spin()` 立即返回；**之后还能再 spin()**。
/// 对齐 rclcpp rolling 的修法（复位 cancel 意图后再进循环）—— 否则"停过之后再启动"这个
/// 合法用法就被我们自己的实现禁掉了。
TEST_F(RobotLifecycleTest, SpinRunsAgainAfterStopIsConsumed) {
  RobotConfig config;
  config.node_name = "lifecycle_restart";
  auto robot = std::make_unique<Robot>(config);

  robot->stop();
  // 第一次 spin() 也放进线程 + 轮询：回归时要么红、要么被断言拦下，**不能挂死**（实测反证过一次）。
  std::atomic<bool> consumed{false};
  std::thread first([&consumed, &robot]() {
    robot->spin();
    consumed.store(true);
  });
  for (int i = 0; i < 40 && !consumed.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const bool consumed_ok = consumed.load();
  EXPECT_TRUE(consumed_ok) << "stop() 早于 spin() 被吃掉（这一次没返回）";
  if (!consumed_ok) {
    first.detach();
    robot.release();  // 同上：不析构正在被线程使用的对象
    return;
  }
  first.join();

  // 再 spin() 应回到"正常阻塞驱动"的行为 ⇒ 300 ms 内**不该**返回。
  std::atomic<bool> returned{false};
  std::thread spinner([&returned, &robot]() {
    robot->spin();
    returned.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(returned.load()) << "stop 被消费后 spin() 仍立即返回（意图被粘住了）";

  robot->stop();      // 收尾：这次是"spin 正在跑"的情形 ⇒ cancel() 直接生效
  spinner.join();
}

/// `spin()` **运行期间**收到 `stop()`：那次 stop 由那次 spin 消费 ⇒ 之后还能再 spin()。
/// 这是清零规则的自洽性检查 —— **`spin()` 返回时 `stop_requested_` 必为 false**。
/// 反证：去掉 `runtime.cpp` 里那句 `store(false)` ⇒ 本用例红（但不会挂死）。
TEST_F(RobotLifecycleTest, StopDuringSpinDoesNotLeakIntoNextSpin) {
  RobotConfig config;
  config.node_name = "lifecycle_stop_during";
  auto robot = std::make_unique<Robot>(config);

  std::atomic<bool> returned{false};
  std::thread spinner([&returned, &robot]() {
    robot->spin();  // 先真的跑起来
    returned.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 让它进入循环
  EXPECT_FALSE(returned.load()) << "spin() 没跑起来，后面的断言就没意义了";

  robot->stop();  // 运行期间叫停 ⇒ cancel() 生效

  for (int i = 0; i < 40 && !returned.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const bool stopped_ok = returned.load();
  EXPECT_TRUE(stopped_ok) << "stop() 之后 spin() 没返回";
  if (!stopped_ok) {
    spinner.detach();
    robot.release();
    return;
  }
  spinner.join();

  // 关键断言：这一次 stop 不该漏给下一次 spin()
  std::atomic<bool> second_returned{false};
  std::thread second([&second_returned, &robot]() {
    robot->spin();
    second_returned.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(second_returned.load()) << "上一次的 stop 漏给了下一次 spin()（该跑却立刻返回）";

  robot->stop();
  second.join();
}

}  // namespace
