#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "robot_sdk/navigation_api.hpp"
#include "robot_sdk/transform.hpp"

using namespace robot_sdk;
using namespace std::chrono_literals;

namespace {

/// 冒烟测试（构造接线 / 无服务端时的退化路径）+ 行为测试（见文件末尾的假服务端）。
/// 行为测试要回答的问题只有一句：服务端**自称成功**时，SDK 凭什么相信它。
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
    // goToAsync 现在要等到"接受"（对齐 nav2）⇒ 没有服务端就得等满这个超时，
    // 默认 5 秒会让用例白等；这里收短，只为了跑得快。
    config.server_wait_timeout_s = 0.2;
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

// 派发阶段就失败：没有服务端 ⇒ 等不到"接受" ⇒ 返回**空指针**（对齐 nav2 的 `return None`）。
TEST_F(NavigationApiTest, GoToAsyncReturnsNullWhenServerDoesNotAccept) {
  auto node = std::make_shared<rclcpp::Node>("nav_api_no_accept");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto transform = std::make_shared<TransformManager>(node);

  NavigationAPI nav(node, group, transform, configWithNoServer());

  NavigationGoal goal;
  goal.target = Pose2D{1.0, 2.0, 0.0};

  EXPECT_EQ(nav.goToAsync(goal), nullptr);
}

// ── 行为测试：假服务端 ───────────────────────────────────────────────
namespace {

using Action = robot_sdk_interfaces::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

/// FakeNavigateServer：一个"会撒谎"的假服务端 —— 接受目标后**不移动任何东西**，直接回 success=true。
/// 用于验证 goTo() 不轻信这个声明 —— 它会读回位姿核实。
class FakeNavigateServer {
 public:
  // 曾经被取消过几次 —— 用来证明"取消真的送到了服务端"，不依赖客户端调 wait()。
  std::atomic<int> cancel_seen{0};

  // hold_s > 0 时 execute 会拖这么久再回成功 —— 好让"取消"有机会送达并被断言到。
  FakeNavigateServer(const rclcpp::Node::SharedPtr& node, const std::string& name,
                     double hold_s = 0.0) {
    server_ = rclcpp_action::create_server<Action>(
        node, name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle>) {
          cancel_seen.fetch_add(1);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [hold_s](const std::shared_ptr<GoalHandle> handle) {
          std::thread([handle, hold_s]() {
            const auto deadline =
                std::chrono::steady_clock::now() + std::chrono::duration<double>(hold_s);
            while (std::chrono::steady_clock::now() < deadline) {
              if (handle->is_canceling()) {
                handle->canceled(std::make_shared<Action::Result>());
                return;
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            // 立刻"成功"，但机器人一步都没动 —— 这就是要复现的那类服务端。
            auto result = std::make_shared<Action::Result>();
            result->success = true;
            result->final_distance_error = 0.0f;
            result->message = "reached target";
            handle->succeed(result);
          }).detach();
        });
  }

 private:
  rclcpp_action::Server<Action>::SharedPtr server_;
};

geometry_msgs::msg::TransformStamped originTransform() {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_footprint";
  tf.transform.rotation.w = 1.0;
  return tf;
}

/// 搭一套：节点 + 假服务端 + 原点处的静态 TF，并返回跑起来的 executor。
struct Harness {
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("nav_api_fake");
  rclcpp::CallbackGroup::SharedPtr group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::shared_ptr<TransformManager> transform = std::make_shared<TransformManager>(node);
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::thread spinner;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
  std::unique_ptr<FakeNavigateServer> server;

  explicit Harness(bool publish_tf, const std::string& action_name, double hold_s = 0.0) {
    if (publish_tf) {
      tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
      tf_broadcaster->sendTransform(originTransform());
    }
    server = std::make_unique<FakeNavigateServer>(node, action_name, hold_s);
    executor->add_node(node);
    spinner = std::thread([this]() { executor->spin(); });
  }
  ~Harness() {
    executor->cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
  }
};

NavigationConfig fakeServerConfig() {
  NavigationConfig config;
  config.action_name = "fake_navigate_to_pose";
  config.arrival_tolerance_m = 0.05;        // 收紧，让判定明确
  config.arrival_yaw_tolerance_rad = 0.05;
  config.server_wait_timeout_s = 2.0;
  return config;
}

NavigationGoal goalAt(double x) {
  NavigationGoal goal;
  goal.target = Pose2D{x, 0.0, 0.0};
  goal.timeout_s = 8.0;
  return goal;
}

TEST_F(NavigationApiTest, GoToRejectsSuccessThatPoseDoesNotSupport) {
  Harness h(/*publish_tf=*/true, "fake_navigate_to_pose");
  NavigationAPI nav(h.node, h.group, h.transform, fakeServerConfig());

  // 机器人实际停在原点，目标在 1 m 外，服务端却说"到了" ⇒ SDK 必须判失败。
  auto result = nav.goTo(goalAt(1.0));

  ASSERT_TRUE(result.ok()) << "传输层应当走完（ok 与 success 是两层）";
  EXPECT_FALSE(result.value().success);
  EXPECT_NE(result.value().message.find("off"), std::string::npos)
      << "报文应说明是位姿不符：" << result.value().message;
}

TEST_F(NavigationApiTest, GoToAcceptsSuccessWhenPoseMatches) {
  Harness h(/*publish_tf=*/true, "fake_navigate_to_pose");
  NavigationAPI nav(h.node, h.group, h.transform, fakeServerConfig());

  // 目标就在原点附近（机器人在那儿）⇒ 同样"撒谎"的服务端，这次不该被判失败。
  auto result = nav.goTo(goalAt(0.0));

  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value().success);
}

TEST_F(NavigationApiTest, GoToKeepsSuccessWhenPoseCannotBeRead) {
  // 故意发一条没人用的坐标系 ⇒ 读不回位姿 ⇒ **不改判**（不能因为查询失败把成功的动作判死）。
  Harness h(/*publish_tf=*/false, "fake_navigate_to_pose");
  auto config = fakeServerConfig();
  config.base_frame = "definitely_no_such_frame";
  NavigationAPI nav(h.node, h.group, h.transform, config);

  auto result = nav.goTo(goalAt(1.0));

  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value().success);
}

// 派发**紧接着**取消：这条路上曾丢过取消（那时 goToAsync 不等接受，cancel() 取不到
// goal handle 就被静默丢掉）。现在派发时就等到接受（对齐 nav2），所以取消一定发得出去。
TEST_F(NavigationApiTest, CancelRightAfterDispatchIsNotLost) {
  Harness h(/*publish_tf=*/true, "fake_navigate_to_pose", /*hold_s=*/1.0);
  NavigationAPI nav(h.node, h.group, h.transform, fakeServerConfig());

  ASSERT_TRUE(nav.serverAvailable(2s));
  auto handle = nav.goToAsync(goalAt(5.0));
  EXPECT_TRUE(handle->cancel());             // ← 紧接着取消：服务端受理

  const auto result = handle->wait(8s);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().code, rclcpp_action::ResultCode::CANCELED);
}

// 取消不该依赖"调用方后来调了 wait()"：只调 cancel()，取消也要送到服务端。
TEST_F(NavigationApiTest, CancelReachesServerWithoutWaiting) {
  Harness h(/*publish_tf=*/true, "fake_navigate_to_pose", /*hold_s=*/1.0);
  NavigationAPI nav(h.node, h.group, h.transform, fakeServerConfig());

  ASSERT_TRUE(nav.serverAvailable(2s));
  auto handle = nav.goToAsync(goalAt(5.0));
  handle->cancel();                          // ← 只取消，**不调 wait()**

  for (int i = 0; i < 100 && h.server->cancel_seen.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_GT(h.server->cancel_seen.load(), 0);
}

// cancel() 会等取消响应：目标已被接受时它应返回 true（服务端受理了这个 goal）。
// goToAsync 现在就等到接受，所以这里不必再显式等一次。
TEST_F(NavigationApiTest, CancelWaitsForResponseWhenAccepted) {
  Harness h(/*publish_tf=*/true, "fake_navigate_to_pose", /*hold_s=*/1.0);
  NavigationAPI nav(h.node, h.group, h.transform, fakeServerConfig());

  ASSERT_TRUE(nav.serverAvailable(2s));
  auto handle = nav.goToAsync(goalAt(5.0));

  EXPECT_TRUE(handle->cancel());                // 受理了 ⇒ true（等到了取消响应）
}

// cancelAll 要能容忍"登记过的句柄已经析构"：会同时走 prune（expired）和"锁外取消"。
TEST_F(NavigationApiTest, CancelAllToleratesDroppedGoalHandles) {
  Harness h(/*publish_tf=*/true, "fake_navigate_to_pose", /*hold_s=*/30.0);
  NavigationAPI nav(h.node, h.group, h.transform, fakeServerConfig());

  ASSERT_TRUE(nav.serverAvailable(2s));
  {
    auto handle = nav.goToAsync(goalAt(5.0));   // 会被接受 ⇒ 有句柄、也被登记
    ASSERT_TRUE(handle);
  }  // ← 句柄在这里析构，登记项随之变成 expired

  EXPECT_NO_THROW(nav.cancelAll());
  EXPECT_NO_THROW(nav.cancelAll());
}

// 所有权回归：把回调组**内联**传进来也必须能正常派发。
// `NavigationAPI` 必须自己持有它 —— `node->create_callback_group()` 返回的组，
// 节点只存 `weak_ptr`（rclcpp 的 `node_base.hpp`），内联传时唯一强引用是临时对象，
// 组会在整条语句结束后析构 ⇒ 挂在它上面的订阅再也不会被 executor 处理。
TEST_F(NavigationApiTest, InlineCallbackGroupStillDispatches) {
  auto node = std::make_shared<rclcpp::Node>("nav_api_inline_group");
  auto transform = std::make_shared<TransformManager>(node);
  auto config = fakeServerConfig();
  config.action_name = "nav_inline_grp";
  config.server_wait_timeout_s = 2.0;

  NavigationAPI nav(node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
                    transform, config);

  auto tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  tf->sendTransform(originTransform());
  FakeNavigateServer server(node, config.action_name, /*hold_s=*/1.0);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(nav.serverAvailable(2s));

  auto handle = nav.goToAsync(goalAt(0.0));
  EXPECT_TRUE(handle != nullptr) << "内联传组也要能派发成功（SDK 必须自己持有那个组）";

  executor.cancel();
  spinner.join();
}

}  // namespace

}  // namespace
