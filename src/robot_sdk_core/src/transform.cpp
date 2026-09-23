#include "robot_sdk/transform.hpp"

#include <string>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_sdk {

TransformManager::TransformManager(const rclcpp::Node::SharedPtr& node)
    : buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
      // spin_thread = true（也是 tf2 的默认值）：让 tf2 **自己**起一个专用线程接收
      // /tf 与 /tf_static。它会同时调用 buffer_->setUsingDedicatedThread(true)，
      // 于是带 timeout 的 lookupTransform 才是被允许、且真的能生效的。
      //
      // 为什么不用 spin_thread = false + 自己调 setUsingDedicatedThread(true)：
      // 那样等于"声称有专用线程，实际没有"。节点上的 /tf 订阅只能靠谁来 spin 节点
      // 才被服务 —— 这在寄生模式（调用方提供节点）下完全不可控：
      // 若调用方用单线程执行器，带 timeout 的查询会**永远超时**（这正是 tf2
      // 在 buffer.hpp 的 threading_error 里警告的情况）。
      // 让 tf2 独占一个线程，则无论谁 spin 节点、执行器有几条线程，都成立。
      listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_, node,
                                                            /*spin_thread=*/true)) {}

Result<geometry_msgs::msg::TransformStamped> TransformManager::lookupTransform(
    const std::string& target_frame, const std::string& source_frame, Duration timeout) const {
  try {
    // tf2::TimePointZero = "我不关心具体时刻，给我最新的"
    return buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero,
                                    tf2::durationFromSec(timeout.count()));
  } catch (const tf2::TransformException& ex) {
    // 异常 → Result。这就是本类的核心职责：
    // 把 tf2 的报错方式翻译成 SDK 统一的错误模型。
    return Error::Unavailable("TF " + source_frame + " -> " + target_frame + ": " + ex.what());
  }
}

Result<geometry_msgs::msg::PoseStamped> TransformManager::transformPose(
    const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame,
    Duration timeout) const {
  // 短路：变换到自己 = 恒等变换，根本不需要查 TF。
  // 这既省一次可能失败的查询，也把"同帧"这个完全合法的输入处理干净。
  if (pose.header.frame_id == target_frame) {
    return pose;
  }
  try {
    geometry_msgs::msg::PoseStamped out;
    buffer_->transform(pose, out, target_frame, tf2::durationFromSec(timeout.count()));
    return out;
  } catch (const tf2::TransformException& ex) {
    return Error::Unavailable("transform to " + target_frame + ": " + ex.what());
  }
}

Result<Pose2D> TransformManager::currentPose(const std::string& target_frame,
                                             const std::string& source_frame,
                                             Duration timeout) const {
  const auto tf = lookupTransform(target_frame, source_frame, timeout);
  if (!tf.ok()) {
    // 错误直接转发 —— Result 的错误是同一套 Error 类型，不用重新包装。
    return tf.error();
  }

  Pose2D pose;
  pose.x = tf.value().transform.translation.x;
  pose.y = tf.value().transform.translation.y;
  // 复用 types.hpp 的 yawFromQuaternion()，而不是在这里重写一遍
  // "Quaternion → Matrix3x3 → getRPY"。同一件事只该有一处实现。
  pose.yaw = yawFromQuaternion(tf.value().transform.rotation);
  return pose;
}

bool TransformManager::canTransform(const std::string& target_frame,
                                    const std::string& source_frame) const {
  return buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero);
}

}  // namespace robot_sdk
