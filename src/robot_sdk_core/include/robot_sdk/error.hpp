#pragma once

#include <string>
#include <utility>

namespace robot_sdk {

/// 稳定、与传输层无关的错误分类。
/// 客户代码应对这些错误码做分支判断，而不是去捕获 ROS 异常。
enum class ErrorCode {
  kOk = 0,
  kTimeout,
  kCancelled,
  kUnavailable,
  kNotInitialized,
  kInvalidArgument,
  kOutOfRange,
  kNotSupported,
  kInternal,
};

inline const char *toString(ErrorCode code) noexcept {
  switch (code) {
  case ErrorCode::kOk:
    return "OK";
  case ErrorCode::kTimeout:
    return "TIMEOUT";
  case ErrorCode::kCancelled:
    return "CANCELLED";
  case ErrorCode::kUnavailable:
    return "UNAVAILABLE";
  case ErrorCode::kNotInitialized:
    return "NOT_INITIALIZED";
  case ErrorCode::kInvalidArgument:
    return "INVALID_ARGUMENT";
  case ErrorCode::kOutOfRange:
    return "OUT_OF_RANGE";
  case ErrorCode::kNotSupported:
    return "NOT_SUPPORTED";
  case ErrorCode::kInternal:
    return "INTERNAL";
  }
  return "UNKNOWN";
}

/// 一个错误值：机器可读的 code + 人类可读的 message。
struct Error {
  ErrorCode code{ErrorCode::kOk};
  std::string message{};

  Error() = default;
  Error(ErrorCode c, std::string m) : code(c), message(std::move(m)) {}

  bool ok() const noexcept { return code == ErrorCode::kOk; }

  /// 显式转换：让 `if (err)` 读作"有错误"。
  /// 故意不用隐式转换，避免有人把 err 当成"成功"来用。
  explicit operator bool() const noexcept { return !ok(); }

  std::string toString() const {
    std::string out = robot_sdk::toString(code);
    if (!message.empty()) {
      out += ": ";
      out += message;
    }
    return out;
  }

  // 具名构造：比 Error{ErrorCode::kTimeout, "..."} 更难写错。
  static Error Ok() { return {}; }
  static Error Timeout(std::string m) {
    return {ErrorCode::kTimeout, std::move(m)};
  }
  static Error Cancelled(std::string m) {
    return {ErrorCode::kCancelled, std::move(m)};
  }
  static Error Unavailable(std::string m) {
    return {ErrorCode::kUnavailable, std::move(m)};
  }
  static Error NotInitialized(std::string m) {
    return {ErrorCode::kNotInitialized, std::move(m)};
  }
  static Error InvalidArgument(std::string m) {
    return {ErrorCode::kInvalidArgument, std::move(m)};
  }
  static Error OutOfRange(std::string m) {
    return {ErrorCode::kOutOfRange, std::move(m)};
  }
  static Error NotSupported(std::string m) {
    return {ErrorCode::kNotSupported, std::move(m)};
  }
  static Error Internal(std::string m) {
    return {ErrorCode::kInternal, std::move(m)};
  }
};

} // namespace robot_sdk