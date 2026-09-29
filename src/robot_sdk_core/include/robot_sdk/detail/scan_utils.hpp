#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace robot_sdk {

/// 在 [angle_min, angle_max] 窗口内求激光扫描的最近有效距离。无可用回波时返回 inf，
/// 调用方可以直接和阈值比较，不必特判"没数据"。无效回波（NaN/inf、量程外）被忽略。
/// 纯函数：不碰节点/时间/全局状态，只吃一个消息结构体，可零成本单测。
inline double nearestObstacleDistance(
    const sensor_msgs::msg::LaserScan &scan,
    double angle_min = -std::numeric_limits<double>::infinity(),
    double angle_max = std::numeric_limits<double>::infinity()) {
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    // 三类"不是障碍"的回波：inf（没打中）、NaN（读数无效）、
    // 以及落在量程外的（< range_min 是盲区；有些驱动用 range_max 表示无回波）。
    if (!std::isfinite(range) || range < scan.range_min ||
        range > scan.range_max) {
      continue;
    }
    // 角度不在消息里，要从起点 + 步长算出来。
    const double angle =
        scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (angle < angle_min || angle > angle_max) {
      continue;
    }
    nearest = std::min(nearest, range);
  }
  return nearest;
}

} // namespace robot_sdk
