#include "robot_sdk/perception_api.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/qos.hpp>

#include "robot_sdk/detail/scan_utils.hpp"
#include "robot_sdk/detail/subscription_wait.hpp"

namespace robot_sdk {

PerceptionAPI::PerceptionAPI(
    const rclcpp::Node::SharedPtr &node,
    const rclcpp::CallbackGroup::SharedPtr &callback_group,
    const PerceptionConfig &config, const SubscriptionQos &sub_qos)
    : node_(node), config_(config), callback_group_(callback_group), qos_(sub_qos) {
  rclcpp::SubscriptionOptions options;
  // ★ 这一行决定所有传感器回调落在哪个回调组：Robot 传的是 sensor 组（Reentrant），
  //   五个高频传感器才并发。忘了传 ⇒ 落进默认组（互斥）⇒ 全部串行、静默失效、运行时不报错。
  options.callback_group = callback_group;

  // QoS 由 Robot 按角色分发进来（= QosConfig::sensor），拼装问 `SubscriptionQos` 要 ——
  // QoS 的知识只在那个类型里。**传感器订阅必须比发布端更宽松**，否则静默收不到（见 docs/qos.md）。
  auto qos = qos_.toQos();

  // 话题名用**相对名**（"scan"），不自己拼前缀 —— rcl 会按节点命名空间展开。
  // 想要别的命名空间就把节点建在那儿，重复实现这套规则只会多出一份不一致的。
  laser_scan_subscription_ = node_->create_subscription<LaserScan>(
      config_.laser_scan_topic, qos,
      [this](LaserScan::ConstSharedPtr msg) { laser_scan_.update(msg); },
      options);

  imu_subscription_ = node_->create_subscription<Imu>(
      config_.imu_topic, qos,
      [this](Imu::ConstSharedPtr msg) { imu_.update(msg); }, options);

  depth_image_subscription_ = node_->create_subscription<Image>(
      config_.depth_image_topic, qos,
      [this](Image::ConstSharedPtr msg) { depth_image_.update(msg); }, options);

  camera_info_subscription_ = node_->create_subscription<CameraInfo>(
      config_.camera_info_topic, qos,
      [this](CameraInfo::ConstSharedPtr msg) { camera_info_.update(msg); },
      options);

  point_cloud_subscription_ = node_->create_subscription<PointCloud2>(
      config_.point_cloud_topic, qos,
      [this](PointCloud2::ConstSharedPtr msg) { point_cloud_.update(msg); },
      options);
}

PerceptionAPI::Duration PerceptionAPI::resolveTimeout(Duration timeout) const {
  // timeout <= 0 表示"用配置的默认值"。
  // 让一个 0 就能表达"我不指定"，调用方不必去查配置里到底是多少。
  if (timeout.count() <= 0.0) {
    return Duration{config_.sensor_timeout_s};
  }
  return timeout;
}

Result<PerceptionAPI::LaserScanPtr>
PerceptionAPI::waitForLaserScan(Duration timeout) {
  return waitForCache(laser_scan_, resolveTimeout(timeout),
                      {*node_, qos_, laser_scan_subscription_->get_topic_name(), "laser scan"});
}

Result<PerceptionAPI::ImuPtr> PerceptionAPI::waitForImu(Duration timeout) {
  return waitForCache(imu_, resolveTimeout(timeout),
                      {*node_, qos_, imu_subscription_->get_topic_name(), "IMU message"});
}

Result<PerceptionAPI::ImagePtr>
PerceptionAPI::waitForDepthImage(Duration timeout) {
  return waitForCache(depth_image_, resolveTimeout(timeout),
                      {*node_, qos_, depth_image_subscription_->get_topic_name(), "depth image"});
}

Result<PerceptionAPI::CameraInfoPtr>
PerceptionAPI::waitForCameraInfo(Duration timeout) {
  return waitForCache(camera_info_, resolveTimeout(timeout),
                      {*node_, qos_, camera_info_subscription_->get_topic_name(), "camera info"});
}

Result<PerceptionAPI::PointCloud2Ptr>
PerceptionAPI::waitForPointCloud(Duration timeout) {
  return waitForCache(point_cloud_, resolveTimeout(timeout),
                      {*node_, qos_, point_cloud_subscription_->get_topic_name(),
                       "point cloud"});
}

double PerceptionAPI::nearestObstacleDistance(double angle_min,
                                              double angle_max) const {
  const auto scan = laser_scan_.latest();
  if (!scan || !*scan) {
    // 语义："还没有数据" = "没发现任何障碍" = 无穷远。
    // 这样调用方可以直接 if (dist < 0.3) 急停，不用先判有没有数据。
    return std::numeric_limits<double>::infinity();
  }
  return robot_sdk::nearestObstacleDistance(**scan, angle_min, angle_max);
}

} // namespace robot_sdk
