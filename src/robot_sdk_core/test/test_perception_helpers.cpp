#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_sdk/detail/scan_utils.hpp"

using namespace robot_sdk;

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

/// angle_min = -1.0, angle_increment = 0.5, 量程 [0.1, 10.0]
/// 五个回波对应角度：-1.0, -0.5, 0.0, 0.5, 1.0
sensor_msgs::msg::LaserScan makeScan() {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -1.0f;
  scan.angle_increment = 0.5f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {kInf, 1.0f, std::nanf(""), 0.05f, 3.0f};
  return scan;
}

}  // namespace

TEST(ScanUtilsTest, NearestIgnoresInvalidReturnsAndOutOfRangeEchoes) {
  const auto scan = makeScan();
  // 回波依次：inf（没打中，忽略）、1.0（有效 ← 最近）、NaN（无效读数，忽略）、
  //           0.05（低于 range_min，盲区内，忽略）、3.0（有效）
  EXPECT_DOUBLE_EQ(1.0, nearestObstacleDistance(scan));
}

TEST(ScanUtilsTest, AngleWindowRestrictsSearch) {
  const auto scan = makeScan();
  // 只看左半区（角度 -1.0 ~ 0.0）→ 只有 index 1 (1.0) 在窗口内
  EXPECT_DOUBLE_EQ(1.0, nearestObstacleDistance(scan, -1.0, 0.0));
  // 只看右半区（角度 0.6 ~ 1.5）→ 只有 index 4 (3.0) 在窗口内
  EXPECT_DOUBLE_EQ(3.0, nearestObstacleDistance(scan, 0.6, 1.5));
  // 窗口内什么都没有 → 无穷大（而不是 0 或 NaN）
  EXPECT_TRUE(std::isinf(nearestObstacleDistance(scan, 5.0, 6.0)));
}

TEST(ScanUtilsTest, ReturnsInfinityWhenNoUsableReturn) {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0f;
  scan.angle_increment = 1.0f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {std::nanf(""), kInf};

  EXPECT_TRUE(std::isinf(nearestObstacleDistance(scan)));
}
