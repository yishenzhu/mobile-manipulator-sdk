#include <gtest/gtest.h>

#include <string>

#include "robot_sdk/error.hpp"

namespace robot_sdk {
namespace {

TEST(ErrorCodeTest, ToStringCoversEveryCode) {
  EXPECT_STREQ("OK", toString(ErrorCode::kOk));
  EXPECT_STREQ("TIMEOUT", toString(ErrorCode::kTimeout));
  EXPECT_STREQ("CANCELLED", toString(ErrorCode::kCancelled));
  EXPECT_STREQ("UNAVAILABLE", toString(ErrorCode::kUnavailable));
  EXPECT_STREQ("NOT_INITIALIZED", toString(ErrorCode::kNotInitialized));
  EXPECT_STREQ("INVALID_ARGUMENT", toString(ErrorCode::kInvalidArgument));
  EXPECT_STREQ("OUT_OF_RANGE", toString(ErrorCode::kOutOfRange));
  EXPECT_STREQ("NOT_SUPPORTED", toString(ErrorCode::kNotSupported));
  EXPECT_STREQ("INTERNAL", toString(ErrorCode::kInternal));
}

TEST(ErrorTest, DefaultConstructedIsOk) {
  const Error err;
  EXPECT_TRUE(err.ok());
  EXPECT_EQ(ErrorCode::kOk, err.code);
  EXPECT_FALSE(static_cast<bool>(err));
}

TEST(ErrorTest, NamedConstructorCarriesCodeAndMessage) {
  const Error err = Error::Timeout("nav took too long");
  EXPECT_FALSE(err.ok());
  EXPECT_EQ(ErrorCode::kTimeout, err.code);
  EXPECT_EQ("nav took too long", err.message);
  EXPECT_TRUE(static_cast<bool>(err));
}

TEST(ErrorTest, ToStringJoinsCodeAndMessage) {
  EXPECT_EQ("TIMEOUT: nav took too long", Error::Timeout("nav took too long").toString());
  EXPECT_EQ("OK", Error::Ok().toString());
}

TEST(ErrorTest, FactoriesMapToDistinctCodes) {
  EXPECT_EQ(ErrorCode::kCancelled, Error::Cancelled("x").code);
  EXPECT_EQ(ErrorCode::kUnavailable, Error::Unavailable("x").code);
  EXPECT_EQ(ErrorCode::kNotInitialized, Error::NotInitialized("x").code);
  EXPECT_EQ(ErrorCode::kInvalidArgument, Error::InvalidArgument("x").code);
  EXPECT_EQ(ErrorCode::kOutOfRange, Error::OutOfRange("x").code);
  EXPECT_EQ(ErrorCode::kNotSupported, Error::NotSupported("x").code);
  EXPECT_EQ(ErrorCode::kInternal, Error::Internal("x").code);
}

}  // namespace
}  // namespace robot_sdk
