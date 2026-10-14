#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_sdk/robot_sdk.hpp"

using namespace robot_sdk;
using namespace std::chrono_literals;

namespace {

/// 三种"收不到"必须给三种不同的答案 —— docs/qos.md §6 的回归测试。
/// 手工验证过的那三条，现在由它守着；否则这行为迟早会腐坏。
class QosDiagnosisTest : public ::testing::Test {
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

  /// ⚠️ 发现是异步的：先等到图里**真的看得见**发布端，再断言。不等的话，"没查到发布端"
  /// 可能只是因为发现还没完成 —— 那是 flake，不是结论。
  static void waitForPublisher(const std::shared_ptr<Robot> &robot, const std::string &topic) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!robot->node()->get_publishers_info_by_topic(topic).empty()) {
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
    ADD_FAILURE() << "5 秒内没发现 " << topic << " 的发布端";
  }

  static RobotConfig configFor(const std::string &node_name, const std::string &topic,
                              const std::string &reliability) {
    RobotConfig config;
    config.node_name = node_name;
    config.perception.laser_scan_topic = topic;
    config.qos.sensor.reliability = reliability;
    return config;
  }
};

TEST_F(QosDiagnosisTest, NoPublisherAtAllIsDiagnosed) {
  const std::string topic = "/qos_test_no_pub";
  auto robot = std::make_shared<Robot>(configFor("qos_no_pub", topic, "best_effort"));

  const auto result = robot->perception().waitForLaserScan(300ms);

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnavailable);
  EXPECT_NE(result.error().message.find("has no publisher at all"), std::string::npos)
      << result.error().message;
}

TEST_F(QosDiagnosisTest, IncompatibleQosIsDiagnosed) {
  const std::string topic = "/qos_test_incompatible";
  auto robot = std::make_shared<Robot>(configFor("qos_incompatible", topic, "reliable"));

  // 发布端是 BEST_EFFORT（SensorDataQoS 就是它），订阅端要 reliable ⇒ 配不上、永远收不到。
  auto sensor = std::make_shared<rclcpp::Node>("qos_incompatible_pub");
  [[maybe_unused]] auto publisher = sensor->create_publisher<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS());
  waitForPublisher(robot, topic);

  const auto result = robot->perception().waitForLaserScan(300ms);

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnavailable);
  // 原因要具体到"谁和谁不配"，而不是一句"不兼容" —— 那正是这功能存在的意义。
  EXPECT_NE(result.error().message.find("incompatible"), std::string::npos);
  EXPECT_NE(result.error().message.find("best_effort"), std::string::npos);
  EXPECT_NE(result.error().message.find("reliable"), std::string::npos);
}

TEST_F(QosDiagnosisTest, CompatibleButSilentStaysTimeout) {
  const std::string topic = "/qos_test_silent";
  auto robot = std::make_shared<Robot>(configFor("qos_silent", topic, "reliable"));

  // 这次发布端也是 reliable ⇒ QoS 兼容，但**一条都不发** ⇒ 只能是纯超时。
  auto sensor = std::make_shared<rclcpp::Node>("qos_silent_pub");
  [[maybe_unused]] auto publisher =
      sensor->create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::QoS(10).reliable());
  waitForPublisher(robot, topic);

  const auto result = robot->perception().waitForLaserScan(300ms);

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kTimeout);
  // 措辞也是契约：查不出别的原因时，一个字都不该变。
  EXPECT_EQ(result.error().message, "no laser scan received within timeout");
}

TEST_F(QosDiagnosisTest, CompatiblePublisherActuallyDelivers) {
  // 正向对照：证明这套设置**真能收到数据**。没有它，上面三条"收不到"可能只是整体坏了。
  const std::string topic = "/qos_test_ok";
  auto robot = std::make_shared<Robot>(configFor("qos_ok", topic, "best_effort"));
  auto sensor = std::make_shared<rclcpp::Node>("qos_ok_pub");
  auto publisher =
      sensor->create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS());
  waitForPublisher(robot, topic);

  std::thread spinner([&robot] { robot->spin(); });
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {1.0f, 2.0f, 3.0f};
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    publisher->publish(scan);
    if (robot->perception().hasLaserScan()) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  const auto result = robot->perception().waitForLaserScan(200ms);
  robot->stop();  // ⚠️ 必须先 stop 再 join：只 join 会一直等下去（spin 不会自己返回）
  spinner.join();

  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value()->ranges.size(), 3u);
}

}  // namespace
