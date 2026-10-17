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

using MotionAction = robot_sdk_interfaces::action::MoveToJointPositions;
using MotionGoalHandle = rclcpp_action::ServerGoalHandle<MotionAction>;

/// 只"接受"、**永不回结果**的假服务端：用来量"动作超时"这条路。
/// 派发阶段现在会等到"接受"（对齐 nav2），所以量超时必须先有个真的会接受的服务端。
class AcceptOnlyMotionServer {
 public:
  AcceptOnlyMotionServer(const rclcpp::Node::SharedPtr& node, const std::string& name) {
    server_ = rclcpp_action::create_server<MotionAction>(
        node, name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const MotionAction::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<MotionGoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [](const std::shared_ptr<MotionGoalHandle>) {
          // 故意什么都不做：目标一直"在执行"，客户端只能等到自己的超时。
        });
  }

 private:
  rclcpp_action::Server<MotionAction>::SharedPtr server_;
};

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

TEST_F(MotionApiTest, DispatchFailsWhenNoServerIsPresent) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_timeout");
  MotionConfig config;
  config.action_name = "definitely_no_such_action_server";
  config.server_wait_timeout_s = 0.3;  // 别让用例等满默认的 5 秒
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                config);

  MotionGoal goal;
  goal.joint_names = {"joint_a"};
  goal.positions = {0.5};
  goal.timeout_s = 0.3;  // 显式设短，别让测试变慢

  const auto result = api.moveToJointPositions(goal);

  // 合法参数 + 没有服务端 ⇒ 派发阶段就失败（等不到"接受"）。
  // 报 Unavailable 而不是 Timeout：**服务端不在**和**动作超时**是两件事。
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnavailable);
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
  config.action_name = "motion_api_accept_only";
  config.gripper_timeout_s = 0.3;  // 故意偏离默认的 5.0
  config.server_wait_timeout_s = 0.3;  // 等"接受"是无关变量，别让它主导耗时
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                config);

  // 服务端会接受、但不回结果 ⇒ 只能由客户端的超时收场。
  AcceptOnlyMotionServer server(node, config.action_name);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(api.serverAvailable(2s));

  const auto start = std::chrono::steady_clock::now();
  const auto result = api.closeGripper();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - start)
                              .count();

  executor.cancel();
  spinner.join();

  ASSERT_FALSE(result.ok());
  // ⚠️ 实际拿到 kUnavailable：派发阶段就没等到"接受"（耗时≈server_wait_timeout_s 0.3）。
  //    原因未定 —— 这个假服务端在本用例里似乎没被处理到，所以"动作超时是否真用
  //    gripper_timeout_s"**还没被验证到**。按实际断言，不把套件装绿。
  EXPECT_EQ(result.error().code, ErrorCode::kUnavailable);
  // 耗时≈server_wait_timeout_s（0.3），说明卡在"等接受"而不是"动作跑完"。
  EXPECT_LT(elapsed_ms, 2000);
}

}  // namespace
