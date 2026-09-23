#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace robot_sdk {

/// 在 [angle_min, angle_max] 角窗口内，求激光扫描的最近有效距离。
///
/// 无效回波（NaN/inf，或落在传感器声明量程之外的）会被忽略。
/// **没有任何可用回波时返回无穷大** —— 这样调用方可以直接和阈值比较，
/// 不必特判"没有数据"这种情况：
///
///     if (robot->perception().nearestObstacleDistance() < 0.3) { /* 急停 */ }
///
/// 注意这是个**纯函数**：不碰节点、不碰时间、不碰全局状态，
/// 只吃一个消息结构体。所以它能被完全单元测试（成本接近零）。
inline double nearestObstacleDistance(
    const sensor_msgs::msg::LaserScan &scan,
    double angle_min = -std::numeric_limits<double>::infinity(),
    double angle_max = std::numeric_limits<double>::infinity()) {
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    // 三种"不是障碍"的回波，都要滤掉：
    //   inf  —— 最常见的"没打中任何东西"
    //   NaN  —— 读数无效（驱动/硬件错误）
    //   < range_min —— 落在传感器盲区内，读数不可信
    //   > range_max —— 有些驱动用 range_max（而非 inf）表示"无回波"
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
