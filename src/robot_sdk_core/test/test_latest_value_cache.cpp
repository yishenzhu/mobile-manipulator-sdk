#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "robot_sdk/latest_value_cache.hpp"

using namespace robot_sdk;
using namespace std::chrono_literals;

TEST(LatestValueCacheTest, EmptyByDefault) {
  const LatestValueCache<int> cache;
  EXPECT_FALSE(cache.hasValue());
  EXPECT_FALSE(cache.latest().has_value());
  EXPECT_EQ(cache.sequence(), 0u);
}

TEST(LatestValueCacheTest, KeepsOnlyMostRecentSample) {
  LatestValueCache<int> cache;
  cache.update(1);
  cache.update(2);
  cache.update(3);

  EXPECT_EQ(*cache.latest(), 3);
  EXPECT_EQ(cache.sequence(), 3u); // 计数器累计到 3，但只留了 1 个值
}

TEST(LatestValueCacheTest, WaitForNextRequiresANewerSample) {
  LatestValueCache<int> cache;
  cache.update(1);

  EXPECT_FALSE(cache.waitForNext(cache.sequence(), 50ms).has_value());

  std::thread producer([&cache] {
    std::this_thread::sleep_for(50ms);
    cache.update(2);
  });
  const auto value = cache.waitForNext(1, 2s);
  producer.join();

  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, 2);
}