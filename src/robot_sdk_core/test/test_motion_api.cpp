#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/motion_api.hpp"

using namespace robot_sdk;
using namespace std::chrono_literals;

namespace {

/// `moveToJointPositions()` 会**先校验参数、再发目标** ——
/// 所以非法参数这条路完全不需要 action server，可以纯本地测。
/// 这类"发出去之前就拦掉"的设计，直接把可测性提高了。
class MotionApiTest : public ::testing::Test {
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

TEST_F(MotionApiTest, RejectsGoalWithNoJointNames) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_no_names");
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                MotionConfig{});

  MotionGoal goal;  // joint_names 与 positions 都是空的
  const auto result = api.moveToJointPositions(goal);

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidArgument);
}

TEST_F(MotionApiTest, RejectsMismatchedJointNamesAndPositions) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_mismatch");
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                MotionConfig{});

  MotionGoal goal;
  goal.joint_names = {"joint_a", "joint_b"};
  goal.positions = {0.1};  // 少一个

  const auto result = api.moveToJointPositions(goal);

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidArgument);
  // 错误信息要说清"对不上"，否则客户看不出问题在哪。
  EXPECT_NE(result.error().message.find("join"), std::string::npos);
}

TEST_F(MotionApiTest, MoveToNamedTargetRejectsUnknownName) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_unknown_target");
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                MotionConfig{});  // named_targets 是空的

  const auto result = api.moveToNamedTarget("no_such_pose");

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidArgument);
}

TEST_F(MotionApiTest, MoveToNamedTargetNeedsJointNamesConfigured) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_no_joints");
  MotionConfig config;
  // 有具名位姿，但没配关节名 —— 无法把"一串位置"对应到具体关节。
  config.named_targets["ready"] = {0.1, 0.2};
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                config);

  const auto result = api.moveToNamedTarget("ready");

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kNotInitialized);
}

TEST_F(MotionApiTest, TimesOutWhenNoServerIsPresent) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_timeout");
  MotionConfig config;
  config.action_name = "definitely_no_such_action_server";
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                config);

  MotionGoal goal;
  goal.joint_names = {"joint_a"};
  goal.positions = {0.5};
  goal.timeout_s = 0.3;  // 显式设短，别让测试变慢

  const auto result = api.moveToJointPositions(goal);

  // 合法参数 + 没有服务端 → 必须是**可解释的超时**，而不是崩溃或永久阻塞。
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kTimeout);
}

TEST_F(MotionApiTest, CancelAllToleratesEmptyRegistry) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_cancel_empty");
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                MotionConfig{});

  EXPECT_NO_THROW(api.cancelAll());
}

/// 这条是「夹爪超时从配置来」这个改动的**守门员**。
/// 如果有人把 `setGripper()` 的超时改回硬编码常量，它会在这里失败。
TEST_F(MotionApiTest, GripperTimeoutComesFromConfig) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_gripper_timeout");
  MotionConfig config;
  config.action_name = "definitely_no_such_action_server";
  config.gripper_timeout_s = 0.3;  // 故意偏离默认的 5.0
  // 没有服务端时，"等接受"也会耗满它自己的预算 —— 那是无关变量，一起固定住，
  // 否则这条用例量到的是等接受的时间，而不是夹爪超时。
  config.server_wait_timeout_s = 0.3;
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                config);

  const auto start = std::chrono::steady_clock::now();
  const auto result = api.closeGripper();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - start)
                              .count();

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kTimeout);
  // 实际等待时间应当接近配置里的 300ms，而不是默认的 5000ms。
  EXPECT_LT(elapsed_ms, 2000);
}

}  // namespace
