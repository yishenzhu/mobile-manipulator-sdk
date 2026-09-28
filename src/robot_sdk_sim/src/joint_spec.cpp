#include "robot_sdk_sim/joint_spec.hpp"

#include <fstream>
#include <sstream>
#include <utility>

#include <urdf/model.h>

namespace robot_sdk::sim {

std::map<std::string, JointSpec> jointSpecsFromFile(const std::string &path,
                                                   const std::vector<std::string> &wanted) {
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  std::ostringstream text;
  text << in.rdbuf();

  urdf::Model model;
  if (!model.initString(text.str())) {
    return {};
  }

  std::map<std::string, JointSpec> specs;
  for (const std::string &name : wanted) {
    const urdf::JointConstSharedPtr joint = model.getJoint(name);
    if (!joint) {
      continue;
    }
    JointSpec spec;
    spec.name = name;
    spec.movable = joint->type != urdf::Joint::FIXED;
    spec.continuous = joint->type == urdf::Joint::CONTINUOUS;
    if (joint->limits) {
      spec.max_velocity = joint->limits->velocity;
    }
    // revolute / prismatic：urdfdom 保证它们**一定**带 <limit>（缺了根本解析不过），
    // 所以这里取 lower/upper 是安全的 —— 不用再判 joint->limits。
    if (spec.limited()) {
      spec.lower = joint->limits->lower;
      spec.upper = joint->limits->upper;
    }
    specs.emplace(name, std::move(spec));
  }
  return specs;
}

} // namespace robot_sdk::sim
