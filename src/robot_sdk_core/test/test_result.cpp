#include <gtest/gtest.h>

#include <string>

#include "robot_sdk/result.hpp"

namespace robot_sdk {
namespace {

TEST(ResultTest, HoldsValueOnSuccess) {
  const Result<int> r{42};
  EXPECT_TRUE(r.ok());
  EXPECT_TRUE(static_cast<bool>(r));
  EXPECT_EQ(42, r.value());
  EXPECT_TRUE(r.error().ok());
}

TEST(ResultTest, HoldsErrorOnFailure) {
  const Result<int> r{Error::Unavailable("server down")};
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_EQ(ErrorCode::kUnavailable, r.error().code);
  EXPECT_EQ("server down", r.error().message);
}

TEST(ResultTest, ValueThrowsSdkExceptionWithSameError) {
  const Result<int> r{Error::Timeout("too slow")};
  try {
    (void)r.value();
    FAIL() << "expected SdkException";
  } catch (const SdkException &e) {
    EXPECT_EQ(ErrorCode::kTimeout, e.error().code);
    EXPECT_EQ("TIMEOUT: too slow", std::string(e.what()));
  }
}

TEST(ResultTest, ValueOrFallsBackOnlyOnError) {
  EXPECT_EQ(7, Result<int>{7}.valueOr(-1));
  EXPECT_EQ(-1, Result<int>{Error::Internal("boom")}.valueOr(-1));
}

TEST(ResultTest, ValueCanBeMovedOutOfTemporary) {
  const std::string out = Result<std::string>{"hello"}.value();
  EXPECT_EQ("hello", out);
}

} // namespace
} // namespace robot_sdk