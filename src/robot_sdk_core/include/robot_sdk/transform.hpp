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
/// 客户永远不该被迫去 catch tf2 的异常，也不该操心 buffer 的时序：
/// 每个方法都返回 Result，并且最多等待 `timeout` 让变换变得可用。
///
/// `spin_thread` 为什么取 false（含一个实测到的析构陷阱）见 `学习笔记.md` 附录 C。
class TransformManager {
 public:
  using Duration = std::chrono::duration<double>;

  /// 内部会给 /tf、/tf_static 建一条**独占的** MutuallyExclusive 回调组 ——
  /// 与 tf2 在 spin_thread=true 时的做法一致：TF 需要的是"不被别人堵"，
  /// 不需要和自己并发。理由见 学习笔记.md 附录 C。
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
  /// /tf 的专用车道。**必须自己持有强引用** —— 节点和执行器都只存 weak_ptr，
  /// 组一旦析构，订阅回调会静默地再也不被调度（不报错、不警告）。
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace robot_sdk
