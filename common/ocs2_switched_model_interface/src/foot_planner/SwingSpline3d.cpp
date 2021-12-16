//
// Created by rgrandia on 03.08.21.
//

#include "ocs2_switched_model_interface/foot_planner/SwingSpline3d.h"

namespace switched_model {

SwingSpline3d::SwingSpline3d(const SwingNode3d& start, const SwingNode3d& mid, const SwingNode3d& end)
    : SwingSpline3d(std::vector<SwingNode3d>{start, mid, end}) {}

SwingSpline3d::SwingSpline3d(const std::vector<SwingNode3d>& nodes) {
  std::vector<SwingNode> x;
  x.reserve(nodes.size());
  std::vector<SwingNode> y;
  y.reserve(nodes.size());
  std::vector<SwingNode> z;
  y.reserve(nodes.size());
  for (const auto& node : nodes) {
    x.push_back({node.time, node.position.x(), node.velocity.x()});
    y.push_back({node.time, node.position.y(), node.velocity.y()});
    z.push_back({node.time, node.position.z(), node.velocity.z()});
  }
  x_ = QuinticSwing(x);
  y_ = QuinticSwing(y);
  z_ = QuinticSwing(z);
}

vector3_t SwingSpline3d::position(scalar_t time) const {
  return {x_.position(time), y_.position(time), z_.position(time)};
}

vector3_t SwingSpline3d::velocity(scalar_t time) const {
  return {x_.velocity(time), y_.velocity(time), z_.velocity(time)};
}

vector3_t SwingSpline3d::acceleration(scalar_t time) const {
  return {x_.acceleration(time), y_.acceleration(time), z_.acceleration(time)};
}

vector3_t SwingSpline3d::jerk(scalar_t time) const {
  return {x_.jerk(time), y_.jerk(time), z_.jerk(time)};
}

}  // namespace switched_model
