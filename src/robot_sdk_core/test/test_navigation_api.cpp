#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/navigation_api.hpp"
#include "robot_sdk/transform.hpp"

using namespace robot_sdk;
using namespace std::chrono_literals;

namespace {

/// 冒烟测试：本类没有真正的 action server 就没法做行为测试，
/// 但"构造接线是否正确、无服务端时的退化路径是否安全"是可以测的 ——
/// 而这类问题（构造崩溃、空表操作出错）恰恰是最容易漏的。
class NavigationApiTest : public ::testing::Test {
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

  static NavigationConfig configWithNoServer() {
    NavigationConfig config;
    // 故意指向一个不存在的 action server，走"后端不在场"的路径。
    config.action_name = "definitely_no_such_action_server";
    return config;
  }
};

TEST_F(NavigationApiTest, ConstructsAndSurvivesEmptyCancelAll) {
  auto node = std::make_shared<rclcpp::Node>("nav_api_smoke");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto transform = std::make_shared<TransformManager>(node);

  NavigationAPI nav(node, group, transform, NavigationConfig{});

  // 登记表为空时，cancelAll() 必须是无害的 no-op。
  EXPECT_NO_THROW(nav.cancelAll());
}

TEST_F(NavigationApiTest, ServerAvailableIsFalseWithoutAServer) {
  auto node = std::make_shared<rclcpp::Node>("nav_api_no_server");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto transform = std::make_shared<TransformManager>(node);

  NavigationAPI nav(node, group, transform, configWithNoServer());

  // 没有服务端时应**快速返回 false**，而不是抛异常或永久阻塞。
  EXPECT_FALSE(nav.serverAvailable(100ms));
}

TEST_F(NavigationApiTest, CancelAllToleratesDroppedGoalHandles) {
  auto node = std::make_shared<rclcpp::Node>("nav_api_dropped");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto transform = std::make_shared<TransformManager>(node);

  NavigationAPI nav(node, group, transform, configWithNoServer());

  {
    NavigationGoal goal;
    goal.target = Pose2D{1.0, 2.0, 0.0};
    // 没有服务端，这个目标会被拒绝 —— 但它仍然会被登记进 active_handles_。
    auto handle = nav.goToAsync(goal);
    ASSERT_TRUE(handle);
  }  // ← 句柄在这里析构，登记项随之变成 expired

  // 这条路径会同时走 prune（expired）和"锁外 cancel"：
  // 必须安全，不能因为句柄已死而出错。
  EXPECT_NO_THROW(nav.cancelAll());
  EXPECT_NO_THROW(nav.cancelAll());
}

}  // namespace
