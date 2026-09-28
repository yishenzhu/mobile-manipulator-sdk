#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "robot_sdk_sim/joint_spec.hpp"

namespace {

using robot_sdk::sim::JointSpec;
using robot_sdk::sim::jointSpecsFromFile;

// 最小 URDF：三种关节类型各一个。写成临时文件 —— 被测函数的入口就是**文件路径**
// （机器人自述是这么交给模拟器的）。
const char *kMiniUrdf = R"XML(<?xml version="1.0"?>
<robot name="mini">
  <link name="base"/>
  <link name="arm"/>
  <link name="tip"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <limit effort="10" lower="-1.5" upper="2.5" velocity="0.75"/>
  </joint>
  <joint name="spinner" type="continuous">
    <parent link="arm"/>
    <child link="tip"/>
  </joint>
  <joint name="glue" type="fixed">
    <parent link="arm"/>
    <child link="tip"/>
  </joint>
</robot>
)XML";

class JointSpecTest : public ::testing::Test {
protected:
  void SetUp() override {
    path_ = testing::TempDir() + "mini.urdf";
    std::ofstream(path_) << kMiniUrdf;
  }
  std::string path_;
};

TEST_F(JointSpecTest, ReadsTypeAndLimits) {
  const auto specs = jointSpecsFromFile(path_, {"shoulder"});
  ASSERT_EQ(1u, specs.size());
  const JointSpec &spec = specs.at("shoulder");
  EXPECT_TRUE(spec.movable);
  EXPECT_FALSE(spec.continuous);
  EXPECT_TRUE(spec.limited());
  EXPECT_DOUBLE_EQ(-1.5, spec.lower);
  EXPECT_DOUBLE_EQ(2.5, spec.upper);
  EXPECT_DOUBLE_EQ(0.75, spec.max_velocity);
}

// 这条守的是那个**恒等关系**：可动 且 不连续 才算"有限位"。
// 也就是 has_limits 那个字段为什么可以不存在。
TEST_F(JointSpecTest, ContinuousIsMovableButNotLimited) {
  // ⚠️ map 要先落在具名变量上：直接对返回值 .at() 取引用会绑到临时对象，
  // 表达式一结束就悬空（-Wdangling-reference 抓到过）。
  const auto specs = jointSpecsFromFile(path_, {"spinner"});
  const JointSpec &spec = specs.at("spinner");
  EXPECT_TRUE(spec.movable);
  EXPECT_TRUE(spec.continuous);
  EXPECT_FALSE(spec.limited());
}

TEST_F(JointSpecTest, FixedIsNotMovable) {
  EXPECT_FALSE(jointSpecsFromFile(path_, {"glue"}).at("glue").movable);
}

TEST_F(JointSpecTest, OnlyWantedJointsComeBack) {
  const auto specs = jointSpecsFromFile(path_, {"shoulder", "spinner"});
  EXPECT_EQ(2u, specs.size());
  EXPECT_EQ(0u, specs.count("glue"));
}

// 读不到 / 解析不了 / 名字不对 —— 一律**空表、不抛**：模拟器少一份描述也该能起来
// （只是不查限位、不折角而已），那是它以前的行为。
TEST_F(JointSpecTest, MissingFileYieldsEmpty) {
  EXPECT_TRUE(jointSpecsFromFile("/nonexistent/robot.urdf", {"shoulder"}).empty());
}

TEST_F(JointSpecTest, BrokenUrdfYieldsEmpty) {
  const std::string bad = testing::TempDir() + "bad.urdf";
  std::ofstream(bad) << "<robot name='x'><joint";
  EXPECT_TRUE(jointSpecsFromFile(bad, {"shoulder"}).empty());
}

TEST_F(JointSpecTest, UnknownNameYieldsEmpty) {
  EXPECT_TRUE(jointSpecsFromFile(path_, {"nope"}).empty());
}

} // namespace
