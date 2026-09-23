#include "robot_sdk/transform.hpp"

#include <string>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/qos.hpp>

namespace robot_sdk {
namespace {

/// /tf（动态）允许用户覆盖的 QoS 策略。与 tf2 监听器的默认值一致 ——
/// 为什么自己写一份而不调用 tf2 的、以及这份设置为什么必须保留，见 学习笔记.md 附录 C。
rclcpp::QosOverridingOptions dynamicQosOverridingOptions() {
  return rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
}

/// /tf_static 允许用户覆盖的 QoS 策略。**故意不含 Durability** —— 原因见 附录 C。
rclcpp::QosOverridingOptions staticQosOverridingOptions() {
  return rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
}

}  // namespace

TransformManager::TransformManager(const rclcpp::Node::SharedPtr& node)
    : buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())) {
  // 允许带 timeout 的阻塞等待：节点的执行器线程会喂 buffer。
  buffer_->setUsingDedicatedThread(true);

  // /tf 的独占车道。与 tf2 在 spin_thread=true 时的做法一致（见 附录 C）。
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 两份 options 不能共用（两份策略不同），且必须显式设 qos_overriding_options，
  // 否则会丢掉 QoS 覆盖能力。详见 附录 C。
  rclcpp::SubscriptionOptions tf_options;
  tf_options.qos_overriding_options = dynamicQosOverridingOptions();
  tf_options.callback_group = callback_group_;

  rclcpp::SubscriptionOptions tf_static_options;
  tf_static_options.qos_overriding_options = staticQosOverridingOptions();
  tf_static_options.callback_group = callback_group_;

  // spin_thread = false：/tf 订阅挂在节点上，由节点的执行器喂数据。
  // 不用 tf2 默认的 true —— 那条路有实测到的析构阻塞，见 附录 C。
  //（第 4/5 个参数是 C++ 位置参数所迫；它们是 tf2 的公开 API，且传的就是默认值。）
  listener_ = std::make_shared<tf2_ros::TransformListener>(
      *buffer_, node, /*spin_thread=*/false, tf2_ros::DynamicListenerQoS(),
      tf2_ros::StaticListenerQoS(), tf_options, tf_static_options);
}

Result<geometry_msgs::msg::TransformStamped>
TransformManager::lookupTransform(const std::string &target_frame,
                                  const std::string &source_frame,
                                  Duration timeout) const {
  try {
    // tf2::TimePointZero = "我不关心具体时刻，给我最新的"
    return buffer_->lookupTransform(target_frame, source_frame,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(timeout.count()));
  } catch (const tf2::TransformException &ex) {
    // 异常 → Result。这就是本类的核心职责：
    // 把 tf2 的报错方式翻译成 SDK 统一的错误模型。
    return Error::Unavailable("TF " + source_frame + " -> " + target_frame +
                              ": " + ex.what());
  }
}

Result<geometry_msgs::msg::PoseStamped>
TransformManager::transformPose(const geometry_msgs::msg::PoseStamped &pose,
                                const std::string &target_frame,
                                Duration timeout) const {
  // 短路：变换到自己 = 恒等变换，根本不需要查 TF。
  // 这既省一次可能失败的查询，也把"同帧"这个完全合法的输入处理干净。
  if (pose.header.frame_id == target_frame) {
    return pose;
  }
  try {
    geometry_msgs::msg::PoseStamped out;
    buffer_->transform(pose, out, target_frame,
                       tf2::durationFromSec(timeout.count()));
    return out;
  } catch (const tf2::TransformException &ex) {
    return Error::Unavailable("transform to " + target_frame + ": " +
                              ex.what());
  }
}

Result<Pose2D> TransformManager::currentPose(const std::string &target_frame,
                                             const std::string &source_frame,
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

bool TransformManager::canTransform(const std::string &target_frame,
                                    const std::string &source_frame) const {
  return buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero);
}

} // namespace robot_sdk
