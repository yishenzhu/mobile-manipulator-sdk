#include "robot_sdk/transform.hpp"

#include <string>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/qos.hpp>

namespace robot_sdk {
namespace {

/// /tf（动态）允许用户覆盖的 QoS 策略。
///
/// 与 tf2 监听器的默认值一致（`tf2_ros/transform_listener.hpp` 里的
/// `detail::get_default_transform_listener_sub_options()`）。
/// 这里**复制一份**而不是调用它 —— 那个函数在 `tf2_ros::detail::`，框架明确
/// 表示不保证稳定；而"升级某个 ROS 发行版后编译不过"是库不该承担的风险。
///
/// 保留这份设置的意义：有些驱动的 /tf 发成 BEST_EFFORT，而 tf2 的默认订阅是
/// RELIABLE，两者不兼容会**完全收不到 TF** —— 那时用户只能靠 QoS 覆盖救回来。
///
/// ⚠️ 升级 ROS 发行版时，请对照上面那个函数确认策略没变。
rclcpp::QosOverridingOptions dynamicQosOverridingOptions() {
  return rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
}

/// /tf_static 允许用户覆盖的 QoS 策略。
///
/// **故意不含 Durability** —— /tf_static 靠 `transient_local`（latched）让
/// **晚启动的订阅者**也能拿到静态变换（如 base_footprint → base_link）。
/// 一旦允许把它覆盖成 VOLATILE，latched 语义就没了，后启动的节点查不到静态变换。
/// （与 tf2 的 `detail::get_default_transform_listener_static_sub_options()` 一致。）
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

  // 给 /tf 一条独占车道：与 tf2 在 spin_thread=true 时的做法一致（它也自建组），
  // 类型同为 MutuallyExclusive —— TF 需要的是"不被别人堵"，不是"和自己并发"。
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 两份 options 不能共用：/tf 与 /tf_static 允许覆盖的策略不同（见上面的注释）。
  // 另外，只要传自己的 SubscriptionOptions，就必须显式设 qos_overriding_options ——
  // rclcpp 的默认值是空的，不设就等于放弃了 QoS 覆盖能力。
  rclcpp::SubscriptionOptions tf_options;
  tf_options.qos_overriding_options = dynamicQosOverridingOptions();
  tf_options.callback_group = callback_group_;

  rclcpp::SubscriptionOptions tf_static_options;
  tf_static_options.qos_overriding_options = staticQosOverridingOptions();
  tf_static_options.callback_group = callback_group_;

  // spin_thread = false：/tf 订阅挂在节点上，由节点的执行器喂数据。
  // 不用 tf2 默认的 true —— 那条路有实测到的析构阻塞，见 学习笔记.md 附录 C。
  //（第 4/5 个参数是 C++ 位置参数所迫：要传到第 6/7 个就得把它们写出来。
  //  它们是 tf2 的**公开** API，且我们传的就是其默认值，无行为改变。）
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
