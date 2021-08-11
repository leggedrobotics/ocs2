//
// Created by rgrandia on 03.08.21.
//

#include "ocs2_switched_model_interface/foot_planner/SwingSpline3d.h"

namespace switched_model {

SwingSpline3d::SwingSpline3d(const SwingNode3d& start, const SwingNode3d& mid, const SwingNode3d& end)
    : x_({start.time, start.position.x(), start.velocity.x()}, {mid.time, mid.position.x(), mid.velocity.x()},
         {end.time, end.position.x(), end.velocity.x()}),
      y_({start.time, start.position.y(), start.velocity.y()}, {mid.time, mid.position.y(), mid.velocity.y()},
         {end.time, end.position.y(), end.velocity.y()}),
      z_({start.time, start.position.z(), start.velocity.z()}, {mid.time, mid.position.z(), mid.velocity.z()},
         {end.time, end.position.z(), end.velocity.z()}) {}

vector3_t SwingSpline3d::position(scalar_t time) const {
  return {x_.position(time), y_.position(time), z_.position(time)};
}

vector3_t SwingSpline3d::velocity(scalar_t time) const {
  return {x_.velocity(time), y_.velocity(time), z_.velocity(time)};
}

vector3_t SwingSpline3d::acceleration(scalar_t time) const {
  return {x_.acceleration(time), y_.acceleration(time), z_.acceleration(time)};
}

}  // namespace switched_model
