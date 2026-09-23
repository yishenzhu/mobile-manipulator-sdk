#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_sdk/result.hpp"
#include "robot_sdk/types.hpp"

namespace robot_sdk {

/// tf2 的薄封装，但把错误模型换掉了。
///
/// 客户永远不该被迫去 catch tf2 的异常、也不该操心 buffer 的时序：
/// 每个方法都返回 Result，并且最多等待 `timeout` 让变换变得可用。
///
/// **这个类的本质是"错误模型的适配层"** ——
/// tf2 用异常报错，SDK 用 Result，这里就是两者的接缝。
class TransformManager {
 public:
  using Duration = std::chrono::duration<double>;

  explicit TransformManager(const rclcpp::Node::SharedPtr& node);

  /// 求 source_frame 在 target_frame 下的位姿。
  /// 注意参数顺序：**先 target，后 source**（tf2 的老规矩，极易搞反）。
  Result<geometry_msgs::msg::TransformStamped> lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      Duration timeout = Duration{0.1}) const;

  /// 把一个带坐标系的位姿转换到另一个坐标系。
  Result<geometry_msgs::msg::PoseStamped> transformPose(
      const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame,
      Duration timeout = Duration{0.1}) const;

  /// 平面位姿版本的查询：等价于 lookupTransform 后丢弃 z/roll/pitch。
  Result<Pose2D> currentPose(const std::string& target_frame, const std::string& source_frame,
                             Duration timeout = Duration{0.1}) const;

  /// 非阻塞地检查变换现在是否可用。
  bool canTransform(const std::string& target_frame, const std::string& source_frame) const;

 private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace robot_sdk
