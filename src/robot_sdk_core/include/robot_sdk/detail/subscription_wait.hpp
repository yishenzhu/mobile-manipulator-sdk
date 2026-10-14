#pragma once

#include <chrono>
#include <exception>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/qos.hpp>

#include "robot_sdk/config.hpp"
#include "robot_sdk/detail/latest_value_cache.hpp"
#include "robot_sdk/result.hpp"

namespace robot_sdk {

/// 等订阅缓存，超时**先查清原因**再报错，而不是一律丢"等不到"。
///
/// 收不到数据有三种完全不同的原因：没有发布端 / 有发布端但 QoS 配不上 / 只是还没来。
/// rclcpp 其实早就知道前两种（它自己会打 WARN），但**代码看不到** —— 一律是超时。
/// 这里把它的信息接到 SDK 的错误面上（见 docs/qos.md §6）。
///
/// ⚠️ 只在**超时路径**上查 —— 成功时一次都不查，所以成功路径零开销。

/// 诊断要的几样东西。打成结构体是为了别把两个 std::string 挨着传：换位置不会报错。
struct SubscriberFacts {
  rclcpp::Node &node;
  const SubscriptionQos &qos;  ///< 我们要求的 QoS（只在超时那一刻才拼成 rclcpp::QoS）
  std::string topic;           ///< 订阅**解析后**的真实话题名（别用配置里的相对名）
  std::string what;            ///< 人话名字，用在错误消息里
};

/// rclcpp 的枚举 → 人话。只翻两个 RxO 策略，够诊断用即可（rcl 没提供 to_string）。
inline const char *reliabilityText(rclcpp::ReliabilityPolicy policy) {
  switch (policy) {
    case rclcpp::ReliabilityPolicy::BestEffort:
      return "best_effort";
    case rclcpp::ReliabilityPolicy::Reliable:
      return "reliable";
    default:
      return "system_default";
  }
}

/// 查"到底为什么收不到"：返回空串 = 没查出别的原因（那就是真的还没来）。
/// ⚠️ get_publishers_info_by_topic 与 qos_check_compatible 都会抛（rclcpp 的 API 就这么定的），
///    在**边界**上接住并降级成"没查出来" —— 诊断失败不该把一次 Result 变成异常。
inline std::string diagnoseSubscription(rclcpp::Node &node, const std::string &topic,
                                       const rclcpp::QoS &wanted) {
  try {
    const auto publishers = node.get_publishers_info_by_topic(topic);
    if (publishers.empty()) {
      return "topic '" + topic + "' has no publisher at all";
    }
    for (const auto &publisher : publishers) {
      const rclcpp::QoSCheckCompatibleResult result =
          rclcpp::qos_check_compatible(publisher.qos_profile(), wanted);
      if (result.compatibility != rclcpp::QoSCompatibility::Error) {
        continue;
      }
      return "topic '" + topic + "' has a publisher (" + publisher.node_name() +
             ") but its QoS is incompatible: " + result.reason + " [publisher reliability=" +
             reliabilityText(publisher.qos_profile().reliability()) +
             ", this subscription wants " + reliabilityText(wanted.reliability()) + "]";
    }
  } catch (const std::exception &) {
    // 见函数注释：诊断失败 = 没诊断，不往外抛。
  }
  return "";
}

template <typename PtrT>
Result<PtrT> waitForCache(LatestValueCache<PtrT> &cache, std::chrono::duration<double> timeout,
                          const SubscriberFacts &facts) {
  const auto value = cache.waitForValue(timeout);
  if (value) {
    return *value;
  }
  const std::string diagnosis = diagnoseSubscription(facts.node, facts.topic, facts.qos.toQos());
  if (!diagnosis.empty()) {
    // 同一个"超时"，原因不同就不该给同一个代码：调用方（例如 agent）才能决定
    // 是去改 QoS，还是去查发布端。查不出来的那种仍报 Timeout（措辞与以前一字不差）。
    return Error::Unavailable(diagnosis);
  }
  return Error::Timeout("no " + facts.what + " received within timeout");
}

} // namespace robot_sdk
