#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/robot_sdk.hpp"  // ← umbrella：整个公开 API，顺带做自包含检查

using namespace robot_sdk;
using namespace std::chrono_literals;

namespace {

/// 只做 "include 整个公开 API" 的编译期检查 —— 顺带充当 umbrella header 的
/// 自包含性测试（谁漏了 include，这里会编译失败）。
/// ⚠️ 套件名必须和下面的 fixture 区分开：gtest 不允许同一套件里混用 TEST 和 TEST_F。
TEST(UmbrellaHeaderTest, IsSelfContained) { SUCCEED(); }

class RobotTest : public ::testing::Test {
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

TEST_F(RobotTest, BuildsItsOwnNodeAndExecutor) {
  RobotConfig config;
  config.node_name = "robot_facade_self";  // 每个用例用不同节点名，避免互相干扰

  auto robot = std::make_shared<Robot>(config);

  ASSERT_TRUE(robot);
  EXPECT_TRUE(robot->ownsExecutor());
  ASSERT_TRUE(robot->node());
  EXPECT_EQ(robot->node()->get_name(), std::string("robot_facade_self"));
}

TEST_F(RobotTest, WorksAsAPlainStackObject) {
  // 持有方式由客户决定：不想要 shared_ptr 就直接放在栈上。
  RobotConfig config;
  config.node_name = "robot_facade_stack";

  Robot robot(config);

  EXPECT_TRUE(robot.ownsExecutor());
  ASSERT_TRUE(robot.node());
  EXPECT_EQ(robot.node()->get_name(), std::string("robot_facade_stack"));
}

TEST_F(RobotTest, AttachingToAnExternalNodeMeansNoOwnedExecutor) {
  auto node = std::make_shared<rclcpp::Node>("robot_facade_external");

  auto robot = std::make_shared<Robot>(node, RobotConfig{});

  ASSERT_TRUE(robot);
  // 寄生模式的核心承诺：**不抢线程**。
  EXPECT_FALSE(robot->ownsExecutor());
  EXPECT_EQ(robot->node().get(), node.get());
}

TEST_F(RobotTest, WiresUpAllFourCapabilityApisFromConfig) {
  RobotConfig config;
  config.node_name = "robot_facade_apis";
  // 注意：两个 action 名**必须不同** —— 它们背后是两种不同的 action 类型，
  // 同名会让底层的 send_goal 服务话题带着两种不兼容的类型撞车，rcl 会拒绝创建。
  // （这正是"同名 action 无法共存"那条规则的推广，见 学习笔记.md。）
  config.navigation.action_name = "no_such_nav_server";
  config.motion.action_name = "no_such_motion_server";

  auto robot = std::make_shared<Robot>(config);

  EXPECT_FALSE(robot->navigation().serverAvailable(50ms));
  EXPECT_FALSE(robot->motion().serverAvailable(50ms));
  // 感知与状态还没收到任何消息，所以是 false（而不是崩溃）。
  EXPECT_FALSE(robot->perception().hasLaserScan());
  EXPECT_FALSE(robot->status().hasStatus());
  // tf() 只是取引用；能拿到就说明装配成功。
  EXPECT_NO_THROW((void)robot->tf());
}

}  // namespace
