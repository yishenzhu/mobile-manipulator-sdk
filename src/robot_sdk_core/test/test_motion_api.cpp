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

/// 会接受、并**立刻回成功结果**的假服务端：覆盖 happy path（派发成功 + wait 拿到终态）。
class SucceedingMotionServer {
 public:
  SucceedingMotionServer(const rclcpp::Node::SharedPtr& node, const std::string& name) {
    server_ = rclcpp_action::create_server<MotionAction>(
        node, name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const MotionAction::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<MotionGoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [](const std::shared_ptr<MotionGoalHandle> handle) {
          std::thread([handle]() {
            handle->succeed(std::make_shared<MotionAction::Result>());
          }).detach();
        });
  }

 private:
  rclcpp_action::Server<MotionAction>::SharedPtr server_;
};

/// 只"接受"、**永不回结果**的假服务端：用来量"动作超时"这条路。
/// ⚠️ 必须**把 handle 存下来**：否则它一出 `handle_accepted` 就析构，rclcpp_action 会把
///    目标当作被取消（实测 message 是 "goal terminated with code CANCELED"），
///    于是 wait() 立刻返回"成功"而不是超时 —— 这样量到的就不是动作超时了。
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
        [this](const std::shared_ptr<MotionGoalHandle> handle) {
          // 存住 ⇒ 目标停在"执行中"，谁也不回结果 ⇒ 客户端只能等自己的超时。
          kept_.push_back(handle);
        });
  }

 private:
  rclcpp_action::Server<MotionAction>::SharedPtr server_;
  std::vector<std::shared_ptr<MotionGoalHandle>> kept_;
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
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  MotionConfig config;
  config.action_name = "motion_api_accept_only";
  config.gripper_timeout_s = 0.3;  // 故意偏离默认的 5.0
  config.server_wait_timeout_s = 2.0;
  MotionAPI api(node, group, config);

  // 服务端会接受、但不回结果 ⇒ 只能由客户端的**动作超时**收场。
  AcceptOnlyMotionServer server(node, config.action_name);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(api.serverAvailable(2s));

  const auto start = std::chrono::steady_clock::now();
  const auto result = api.closeGripper();  // 走 setGripper ⇒ 用 gripper_timeout_s
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - start)
                              .count();

  executor.cancel();
  spinner.join();

  ASSERT_FALSE(result.ok());
  // 派发已经成功（接受拿到了）⇒ 这里的失败必须是**动作超时**，而不是别的东西；
  // 而耗时应当接近配置里的 300ms，而不是默认的 5000ms。
  EXPECT_EQ(result.error().code, ErrorCode::kTimeout);
  EXPECT_LT(elapsed_ms, 2000);
}

/// happy path：服务端会接受**并回结果** ⇒ 派发拿到句柄、wait() 能拿到终态。
TEST_F(MotionApiTest, DispatchAndWaitSucceedAgainstARealServer) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_happy");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  MotionConfig config;
  config.action_name = "motion_api_succeeding";
  config.joint_names = {"joint_a"};
  MotionAPI api(node, group, config);

  SucceedingMotionServer server(node, config.action_name);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(api.serverAvailable(2s));

  MotionGoal goal;
  goal.joint_names = {"joint_a"};
  goal.positions = {0.5};
  goal.timeout_s = 5.0;

  auto handle = api.moveToJointPositionsAsync(goal);
  ASSERT_TRUE(handle != nullptr) << "服务端会接受，句柄不该为空";

  const auto wrapped = handle->wait(5s);
  ASSERT_TRUE(wrapped.ok());

  executor.cancel();
  spinner.join();
}

/// 所有权回归：把回调组**内联**传进来也必须能正常派发。
/// `MotionAPI` 必须自己持有它 —— `node->create_callback_group()` 返回的组，
/// 节点只存 `weak_ptr`（rclcpp 的 `node_base.hpp`），内联传时唯一强引用是临时对象，
/// 组会在整条语句结束后析构 ⇒ 挂在它上面的订阅再也不会被 executor 处理。
TEST_F(MotionApiTest, InlineCallbackGroupStillDispatches) {
  auto node = std::make_shared<rclcpp::Node>("motion_api_inline_group");
  MotionConfig config;
  config.action_name = "motion_api_inline_grp";
  config.joint_names = {"joint_a"};
  MotionAPI api(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                config);

  SucceedingMotionServer server(node, config.action_name);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(api.serverAvailable(2s));

  MotionGoal goal;
  goal.joint_names = {"joint_a"};
  goal.positions = {0.5};
  goal.timeout_s = 5.0;

  auto handle = api.moveToJointPositionsAsync(goal);
  EXPECT_TRUE(handle != nullptr) << "内联传组也要能派发成功（SDK 必须自己持有那个组）";

  executor.cancel();
  spinner.join();
}

}  // namespace
