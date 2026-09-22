#pragma once

#include <cmath>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace robot_sdk {

/// 高层 API 使用的平面位姿。
/// 在边界上保持一个简单的值类型，客户就永远不用为"开到这里"这种
/// 常见需求去碰四元数。
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// 把角度归一到 [-pi, pi)。
inline double normalizeAngle(double angle) {
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0)
    wrapped += 2.0 * M_PI;
  return wrapped - M_PI;
}

/// a 与 b 之间的最短带符号角差，落在 (-pi, pi]。
inline double angleDifference(double a, double b) {
  return normalizeAngle(a - b);
}

inline double distance(const Pose2D &a, const Pose2D &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  tf2::Quaternion tf_q;
  tf_q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion q;
  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();
  return q;
}

inline Pose2D toPose2D(const geometry_msgs::msg::Pose &pose) {
  Pose2D out;
  out.x = pose.position.x;
  out.y = pose.position.y;
  out.yaw = yawFromQuaternion(pose.orientation);
  return out;
}

inline geometry_msgs::msg::Pose toPoseMsg(const Pose2D &pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.position.z = 0.0;
  msg.orientation = quaternionFromYaw(pose.yaw);
  return msg;
}

inline Pose2D toPose2D(const geometry_msgs::msg::PoseStamped &stamped) {
  return toPose2D(stamped.pose);
}

} // namespace robot_sdk