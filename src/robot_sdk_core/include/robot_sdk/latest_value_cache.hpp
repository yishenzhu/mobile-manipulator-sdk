#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

namespace robot_sdk {

/// 线程安全的"最新样本"容器，用于流式 topic。
///
/// 传感器 topic 的发布速率远高于客户代码的消费速率，所以 SDK 暴露
/// "最新值"语义，而不是让客户去消化一堆过期消息。
template <typename T> class LatestValueCache {
public:
  void update(const T &value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      value_ = value;
      ++sequence_;
    } //
    cv_.notify_all();
  }

  std::optional<T> latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  bool hasValue() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_.has_value();
  }

  std::uint64_t sequence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sequence_;
  }

  /// 阻塞直到"曾经收到过至少一个样本"。
  template <typename Rep, typename Period>
  std::optional<T>
  waitForValue(const std::chrono::duration<Rep, Period> &timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this] { return sequence_ > 0; })) {
      return std::nullopt;
    }
    return value_;
  }

  /// 阻塞直到"第 since_sequence 号之后有新样本到达"。
  template <typename Rep, typename Period>
  std::optional<T>
  waitForNext(std::uint64_t since_sequence,
              const std::chrono::duration<Rep, Period> &timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this, since_sequence] {
          return sequence_ > since_sequence;
        })) {
      return std::nullopt;
    }
    return value_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> value_{};
  std::uint64_t sequence_{0};
};

} // namespace robot_sdk