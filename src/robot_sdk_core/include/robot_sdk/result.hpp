#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "robot_sdk/error.hpp"

namespace robot_sdk {

/// 当 Result 里装的是错误、而调用者却调用了 value() 时抛出。
/// 这是"安全网"，不是常规错误通道 —— 常规错误请走 ok()/error()。
class SdkException : public std::runtime_error {
public:
  explicit SdkException(Error error)
      : std::runtime_error(error.toString()), error_(std::move(error)) {}

  const Error &error() const noexcept { return error_; }

private:
  Error error_;
};

/// 要么是 T，要么是 Error。
/// SDK 的公开接口从不抛异常，调用者必须显式检查 ok() / error()。
template <typename T> class Result {
public:
  Result(T value) : value_(std::move(value)) {}
  Result(Error error) : error_(std::move(error)) {}

  bool ok() const noexcept { return value_.has_value(); }
  explicit operator bool() const noexcept { return ok(); }

  const Error &error() const noexcept { return error_; }

  // 三个重载：左值 / const 左值 / 右值。
  // 右值版本能把内部的值 move 出来，避免一次多余的拷贝。
  T &value() & {
    if (!value_)
      throw SdkException(error_);
    return *value_;
  }
  const T &value() const & {
    if (!value_)
      throw SdkException(error_);
    return *value_;
  }
  T &&value() && {
    if (!value_)
      throw SdkException(error_);
    return std::move(*value_);
  }

  /// 失败时给个默认值，适合"拿不到就用兜底值"的场景。
  const T &valueOr(const T &fallback) const noexcept {
    return value_ ? *value_ : fallback;
  }

private:
  std::optional<T> value_{};
  Error error_{};
};

} // namespace robot_sdk