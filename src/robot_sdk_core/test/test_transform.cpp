#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "robot_sdk/transform.hpp"
#include "robot_sdk/types.hpp"

using namespace robot_sdk;
using namespace std::chrono_literals;

namespace {

/// 在后台 spin 一个节点，析构时把线程收干净。需要它是因为 TransformManager 用
/// spin_thread=false，/tf 订阅挂在节点上，得有人 spin（生产里是 RobotRuntime 的执行器）。
/// ⚠️ 用 RAII 而不是手动收尾：gtest 的 ASSERT_* 会直接 return，手动清理会被跳过。
class NodeSpinner {
 public:
  explicit NodeSpinner(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    executor_.add_node(node_);
    thread_ = std::thread([this] { executor_.spin(); });
  }
  ~NodeSpinner() {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  NodeSpinner(const NodeSpinner&) = delete;
  NodeSpinner& operator=(const NodeSpinner&) = delete;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

class TransformTest : public ::testing::Test {
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

/// 同帧变换是恒等变换 —— 这条路径完全不碰 TF，所以不需要任何 TF 数据就能测。
/// 也正因为不碰 TF，它是"最便宜的测试"：构造对象 + 一次调用。
TEST_F(TransformTest, TransformPoseToItsOwnFrameIsIdentity) {
  auto node = std::make_shared<rclcpp::Node>("tf_identity_test");
  TransformManager tf(node);

  geometry_msgs::msg::PoseStamped in;
  in.header.frame_id = "map";
  in.pose.position.x = 1.5;
  in.pose.position.y = -2.0;
  in.pose.position.z = 0.7;

  const auto out = tf.transformPose(in, "map");

  ASSERT_TRUE(out.ok()) << out.error().toString();
  EXPECT_EQ(out.value().header.frame_id, "map");
  EXPECT_DOUBLE_EQ(out.value().pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(out.value().pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(out.value().pose.position.z, 0.7);
}

/// 不存在的坐标系：tf2 会抛异常，我们的封装必须把它转成 Error，
/// 而不是让异常穿透出去（那是 SDK 的硬约束）。
TEST_F(TransformTest, MissingFrameBecomesUnavailableError) {
  auto node = std::make_shared<rclcpp::Node>("tf_missing_test");
  TransformManager tf(node);

  const auto res = tf.lookupTransform("map", "no_such_frame_at_all", 100ms);

  EXPECT_FALSE(res.ok());
  EXPECT_EQ(res.error().code, ErrorCode::kUnavailable);
  EXPECT_FALSE(res.error().message.empty());  // 必须带上 tf2 的原始原因

  EXPECT_FALSE(tf.canTransform("map", "no_such_frame_at_all"));
}

/// 正向路径：真发一个静态变换，验证能查出来。
/// 这是"查不到"之外的另一半 —— 光测失败路径不算覆盖。
TEST_F(TransformTest, LookupSucceedsWithPublishedStaticTransform) {
  auto node = std::make_shared<rclcpp::Node>("tf_static_test");
  // 先把节点的执行器起起来，/tf 订阅才有线程去收。
  NodeSpinner spinner(node);

  TransformManager tf(node);
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "tf_test_parent";
  t.child_frame_id = "tf_test_child";
  t.transform.translation.x = 3.0;
  t.transform.translation.y = 4.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;

  // 轮询 + **重发**，最多约 10 秒。要重发是因为 DDS 发现延迟不受我们控制：只发一次
  // 再死等，会在发现慢时偶发失败（"跑一百次挂一次"最消耗信任）。反复发把发现延迟变成
  // "多等几轮"而不是失败，而正常情况第一轮就成功，几乎不花时间。
  bool ready = false;
  for (int i = 0; i < 50 && !ready; ++i) {
    broadcaster->sendTransform(t);
    std::this_thread::sleep_for(200ms);
    ready = tf.canTransform("tf_test_parent", "tf_test_child");
  }
  ASSERT_TRUE(ready) << "静态变换在约 10 秒内未到达 TF buffer";

  const auto pose = tf.currentPose("tf_test_parent", "tf_test_child", 1s);
  ASSERT_TRUE(pose.ok()) << pose.error().toString();
  EXPECT_NEAR(pose.value().x, 3.0, 1e-9);
  EXPECT_NEAR(pose.value().y, 4.0, 1e-9);
  EXPECT_NEAR(pose.value().yaw, 0.0, 1e-9);
}

}  // namespace
