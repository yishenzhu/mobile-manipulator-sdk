#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_sdk/config.hpp"

using namespace robot_sdk;

TEST(ConfigTest, DefaultsAreUsable) {
  const RobotConfig cfg;
  EXPECT_EQ(cfg.navigation.action_name, "navigate_to_pose");
  EXPECT_EQ(cfg.navigation.global_frame, "map");
  EXPECT_EQ(cfg.navigation.base_frame, "base_footprint");
  EXPECT_EQ(cfg.motion.action_name, "move_to_joint_positions");
  EXPECT_EQ(cfg.status.status_topic, "robot_status");
  EXPECT_EQ(cfg.perception.laser_scan_topic, "scan");
  EXPECT_GT(cfg.navigation.default_timeout_s, 0.0);
  EXPECT_GT(cfg.perception.sensor_timeout_s, 0.0);
}

TEST(ConfigTest, NestedStructsAreDefaultConstructed) {
  const RobotConfig cfg;
  EXPECT_TRUE(cfg.motion.joint_names.empty());    // 空 vector，不是未初始化的垃圾
  EXPECT_TRUE(cfg.motion.named_targets.empty());  // 空 map
  EXPECT_TRUE(cfg.robot_namespace.empty());       // 空 string
  EXPECT_FALSE(cfg.use_sim_time);
  EXPECT_EQ(cfg.motion.gripper_closed_position, 0.0);
}

TEST(ConfigTest, NormalizeNamespaceProducesCanonicalForm) {
  // ROS 2 规则：namespace 不能以 '/' 结尾，除非它只有 '/'。
  // 违反的例子 "/robot1/" 会让 rclcpp::Node 抛 InvalidNamespaceException，
  // 也就是节点根本建不起来 —— 所以这个函数是安全网。
  EXPECT_EQ(normalizeNamespace(""), "");
  EXPECT_EQ(normalizeNamespace("robot1"), "/robot1");
  EXPECT_EQ(normalizeNamespace("/robot1"), "/robot1");
  EXPECT_EQ(normalizeNamespace("/robot1/"), "/robot1");
  EXPECT_EQ(normalizeNamespace("robot1///"), "/robot1");
  EXPECT_EQ(normalizeNamespace("/"), "");    // 根 → 空串（等价）
  EXPECT_EQ(normalizeNamespace("//"), "");   // 非法输入 → 合法结果
  EXPECT_EQ(normalizeNamespace("///"), "");
}

/// fromNode 需要一个真的 rclcpp::Node，所以要初始化 rclcpp。
/// 注意用 SetUpTestSuite / TearDownTestSuite（整个 fixture 各一次），
/// 而不是 SetUp / TearDown（每个 TEST_F 各一次）——
/// rclcpp::init 只能调一次。
class ConfigNodeTest : public ::testing::Test {
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

TEST_F(ConfigNodeTest, ReadsParameterOverridesFromLaunchFile) {
  // parameter_overrides 模拟 launch 文件传参，不用真的起 launch。
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("navigation.global_frame", "odom"),
      rclcpp::Parameter("navigation.default_timeout_s", 12.5),
      rclcpp::Parameter("use_sim_time", true),
  });
  auto node = std::make_shared<rclcpp::Node>("robot_sdk_config_test", options);

  const auto cfg = RobotConfig::fromNode(*node);

  EXPECT_EQ(cfg.node_name, "robot_sdk_config_test");  // 取自节点，不是默认值
  EXPECT_EQ(cfg.navigation.global_frame, "odom");     // 被覆盖
  EXPECT_DOUBLE_EQ(cfg.navigation.default_timeout_s, 12.5);
  EXPECT_TRUE(cfg.use_sim_time);                      // 预声明的参数也能正确读出
  EXPECT_EQ(cfg.navigation.base_frame, "base_footprint");  // 没覆盖的保持默认
}

TEST_F(ConfigNodeTest, ReadsEveryParameterGroup) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("robot_namespace", "robot1"),
      // 字符串数组参数 —— 和标量是不同的代码路径
      rclcpp::Parameter("motion.joint_names", std::vector<std::string>{"joint_a", "joint_b"}),
      rclcpp::Parameter("motion.gripper_open_position", 0.08),
      rclcpp::Parameter("perception.laser_scan_topic", "front/scan"),
      rclcpp::Parameter("perception.queue_depth", 3),
      rclcpp::Parameter("status.status_topic", "health"),
  });
  auto node = std::make_shared<rclcpp::Node>("robot_sdk_config_groups_test", options);

  const auto cfg = RobotConfig::fromNode(*node);

  ASSERT_EQ(cfg.motion.joint_names.size(), 2u);
  EXPECT_EQ(cfg.motion.joint_names[0], "joint_a");
  EXPECT_EQ(cfg.motion.joint_names[1], "joint_b");
  EXPECT_DOUBLE_EQ(cfg.motion.gripper_open_position, 0.08);
  EXPECT_EQ(cfg.perception.laser_scan_topic, "front/scan");
  EXPECT_EQ(cfg.perception.queue_depth, 3);
  EXPECT_EQ(cfg.status.status_topic, "health");
  EXPECT_EQ(cfg.robot_namespace, "robot1");
  // 没覆盖的组保持默认
  EXPECT_EQ(cfg.status.get_status_service, "get_robot_status");
  EXPECT_EQ(cfg.perception.imu_topic, "imu");
}

TEST_F(ConfigNodeTest, NormalizedNamespaceIsAcceptedByRclcpp) {
  // 决定性验证：归一化后的结果必须是 rclcpp 能接受的。
  // 否则节点构造会抛 InvalidNamespaceException —— 整个 SDK 起不来。
  // 对照组："/robot1/" 和 "//" 原样传给 rclcpp 是会抛异常的。
  int index = 0;
  for (const std::string raw : {"robot1", "/robot1", "/robot1/", "robot1///", "/", "//"}) {
    const std::string ns = normalizeNamespace(raw);
    rclcpp::NodeOptions options;
    EXPECT_NO_THROW({
      auto node = std::make_shared<rclcpp::Node>("ns_check_" + std::to_string(index), ns, options);
      node.reset();
    }) << "raw = " << raw << ", normalized = " << ns;
    ++index;
  }
}

/// 反向验证：证明"为什么必须有 normalizeNamespace"。
/// 这条测试故意传入非法命名空间，断言它确实会抛 ——
/// 如果哪天 rclcpp 变宽松了，这条会失败，提醒我们这个安全网可以重新评估。
TEST_F(ConfigNodeTest, RawTrailingSlashNamespaceThrows) {
  rclcpp::NodeOptions options;
  EXPECT_THROW(std::make_shared<rclcpp::Node>("ns_bad_trailing", "/robot1/", options),
               std::exception);
  EXPECT_THROW(std::make_shared<rclcpp::Node>("ns_bad_double", "//", options), std::exception);
}

/// 这条测试守住的是 SDK 的**前提**。
///
/// SDK 现在把相对话题名（"scan"）直接交给 rcl，依赖它按节点命名空间
/// 自动展开 —— 自己不再拼前缀。所以必须把 rcl 的这个行为钉住：
/// 它一旦变了，SDK 的行为会**静默**改变。
///
/// 这也解释了为什么"想要不同命名空间"的正确做法是**建一个该命名空间的节点**，
/// 而不是逐个名字去覆盖前缀。
TEST_F(ConfigNodeTest, RelativeTopicNamesExpandUsingNodeNamespace) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("topic_ns_probe", "/robot1", options);

  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [](sensor_msgs::msg::LaserScan::ConstSharedPtr) {});
  EXPECT_EQ(std::string(sub->get_topic_name()), "/robot1/scan");

  // 多级相对名同样跟随命名空间
  auto nested = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "camera/depth", rclcpp::SensorDataQoS(), [](sensor_msgs::msg::LaserScan::ConstSharedPtr) {});
  EXPECT_EQ(std::string(nested->get_topic_name()), "/robot1/camera/depth");

  // 绝对名不受节点命名空间影响
  auto absolute = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/global/scan", rclcpp::SensorDataQoS(), [](sensor_msgs::msg::LaserScan::ConstSharedPtr) {});
  EXPECT_EQ(std::string(absolute->get_topic_name()), "/global/scan");
}
