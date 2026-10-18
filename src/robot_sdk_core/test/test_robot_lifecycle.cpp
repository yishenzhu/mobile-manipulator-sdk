#include <gtest/gtest.h>

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
    // ⚠️ 必须先确认 spin() 已经进入循环再 stop()：`stop()` 就是 `executor.cancel()`，
    //    而 rclcpp 的 `spin()` 会重置那个状态 ⇒ **cancel 早于 spin 进入 ⇒ stop 被吃掉、
    //    spin 永久阻塞**（实测过：不加这句就挂死）。这里只是让 spin 落位，不是同步业务状态。
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

}  // namespace
