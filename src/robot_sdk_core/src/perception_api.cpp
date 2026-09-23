#include "robot_sdk/perception_api.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/qos.hpp>

#include "robot_sdk/scan_utils.hpp"

namespace robot_sdk {
namespace {

/// 五个 waitForXxx 的唯一差别是「缓存 + 名字」，所以抽成一个模板。
/// （公开 API 保持逐一列出 —— 显式的接口列表比"一个万能函数"更好用，
///   但实现没必要复制五遍。）
///
/// 名字参数用 const std::string&：
///   - 函数只读它、不保存它 → 引用绑定安全（参数绑定的临时对象活到函数返回）
///   - 调用点传字面量会构造一个临时 std::string，短标签落在 SSO 内、无堆分配
///   - 明确"不拥有"，比传值少一次所有权语义的思考
template <typename PtrT>
Result<PtrT> waitForCache(LatestValueCache<PtrT> &cache,
                          PerceptionAPI::Duration timeout,
                          const std::string &what) {
  auto value = cache.waitForValue(timeout);
  if (!value) {
    return Error::Timeout("no " + what + " received within timeout");
  }
  return *value;
}

} // namespace

PerceptionAPI::PerceptionAPI(
    const rclcpp::Node::SharedPtr &node,
    const rclcpp::CallbackGroup::SharedPtr &callback_group,
    const PerceptionConfig &config)
    : node_(node), config_(config) {
  rclcpp::SubscriptionOptions options;
  // ★ 这一行决定了所有传感器回调落在哪个回调组里。
  //   Robot 传进来的是 sensor 组（Reentrant），于是五个高频传感器可以并发。
  //   忘了传它 → 回调落入默认组（MutuallyExclusive）→ 全部串行，
  //   精心设计的并发隔离静默失效，而且运行时不报错。
  options.callback_group = callback_group;

  auto qos = rclcpp::SensorDataQoS();
  // 把配置里的 queue_depth 真正接到 QoS 上。
  // 注意 rclcpp::SensorDataQoS() 的默认深度是 5，与配置默认值 10 并不相同 ——
  // 不写这一行的话，perception.queue_depth 就是个"改了不生效"的死配置
  // （launch 文件里写 50 也不会有任何变化）。
  qos.keep_last(static_cast<std::size_t>(config_.queue_depth));

  // 话题名直接用**相对名**（"scan" 这种），不自己拼前缀。
  // rcl 会在创建订阅时按节点命名空间自动展开成 "/robot1/scan" ——
  // 这是 ROS 2 的内建机制，重复实现它只会多出一套可能不一致的规则。
  // 想要不同的命名空间，正确做法是把节点建在那个命名空间里。
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
  return waitForCache(laser_scan_, resolveTimeout(timeout), "laser scan");
}

Result<PerceptionAPI::ImuPtr> PerceptionAPI::waitForImu(Duration timeout) {
  return waitForCache(imu_, resolveTimeout(timeout), "IMU message");
}

Result<PerceptionAPI::ImagePtr>
PerceptionAPI::waitForDepthImage(Duration timeout) {
  return waitForCache(depth_image_, resolveTimeout(timeout), "depth image");
}

Result<PerceptionAPI::CameraInfoPtr>
PerceptionAPI::waitForCameraInfo(Duration timeout) {
  return waitForCache(camera_info_, resolveTimeout(timeout), "camera info");
}

Result<PerceptionAPI::PointCloud2Ptr>
PerceptionAPI::waitForPointCloud(Duration timeout) {
  return waitForCache(point_cloud_, resolveTimeout(timeout), "point cloud");
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
