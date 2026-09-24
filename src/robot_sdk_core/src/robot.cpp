#include "robot_sdk/robot.hpp"

#include <memory>
#include <utility>

// RobotRuntime 的完整定义只有这个 .cpp 需要 —— 头文件里只前向声明。
#include "robot_sdk/detail/runtime.hpp"

namespace robot_sdk {

// ---------------------------------------------------------------------------
// 唯一的装配点。两个公开构造都委托到这里。
// ---------------------------------------------------------------------------
Robot::Robot(std::shared_ptr<RobotRuntime> runtime) : runtime_(std::move(runtime)) {
  const RobotConfig& cfg = runtime_->config();
  const auto& node = runtime_->node();

  // TransformManager 自建一条独占回调组，所以只收 node。
  transform_ = std::make_shared<TransformManager>(node);

  // 每个 API 只拿**自己那一份配置**（接口隔离）。
  // 各自拿哪条回调组，就是 robot.hpp 类注释里那张并发契约表的落点。
  navigation_ = std::make_unique<NavigationAPI>(node, runtime_->controlGroup(), transform_,
                                                cfg.navigation);
  motion_ = std::make_unique<MotionAPI>(node, runtime_->controlGroup(), cfg.motion);
  perception_ = std::make_unique<PerceptionAPI>(node, runtime_->sensorGroup(), cfg.perception);
  status_ = std::make_unique<StatusAPI>(node, runtime_->statusGroup(), cfg.status);
}

// ---------------------------------------------------------------------------
// 两个公开构造：各自只决定"runtime 从哪来"，装配逻辑复用上面那一处。
// ---------------------------------------------------------------------------
Robot::Robot(const RobotConfig& config) : Robot(std::make_shared<RobotRuntime>(config)) {}

Robot::Robot(const rclcpp::Node::SharedPtr& node, const RobotConfig& config)
    : Robot(std::make_shared<RobotRuntime>(node, config)) {}

// ---------------------------------------------------------------------------
// 要碰 RobotRuntime 的访问器（定义在这里才看得到完整类型）
// ---------------------------------------------------------------------------
const RobotConfig& Robot::config() const { return runtime_->config(); }

const rclcpp::Node::SharedPtr& Robot::node() const { return runtime_->node(); }

bool Robot::ownsExecutor() const { return runtime_->ownsExecutor(); }

// ---------------------------------------------------------------------------
// 转发到 runtime
// ---------------------------------------------------------------------------
void Robot::spin() { runtime_->spin(); }

void Robot::spinSome(std::chrono::milliseconds timeout) { runtime_->spinSome(timeout); }

void Robot::stop() { runtime_->stop(); }

}  // namespace robot_sdk
