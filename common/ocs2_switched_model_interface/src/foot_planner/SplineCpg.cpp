//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"

namespace switched_model {

SplineCpg::SplineCpg(SplineCpgSettings settings) : settings_(settings), midTime_(0.0) {}

void SplineCpg::set(Point liftoff, Point touchdown, scalar_t midHeight) {
  midTime_ = 0.5 * (liftoff.time + touchdown.time);

  CubicSpline::Node start{liftoff.time, liftoff.height, settings_.liftOffVelocity};
  CubicSpline::Node mid{midTime_, midHeight, 0.0};
  CubicSpline::Node end{touchdown.time, touchdown.height, settings_.touchDownVelocity};
  leftSpline_ = CubicSpline(start, mid);
  rightSpline_ = CubicSpline(mid, end);
}

SplineCpg::scalar_t SplineCpg::position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
}

SplineCpg::scalar_t SplineCpg::velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
}

SplineCpg::scalar_t SplineCpg::acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
}

SplineCpg::scalar_t SplineCpg::startTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return leftSpline_.startTimeDerivative(time) + 0.5 * leftSpline_.startTimeDerivative(time);
  } else {
    return 0.5 * rightSpline_.startTimeDerivative(time);
  }
}

SplineCpg::scalar_t SplineCpg::finalTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return 0.5 * leftSpline_.finalTimeDerivative(time);
  } else {
    return rightSpline_.finalTimeDerivative(time) + 0.5 * rightSpline_.finalTimeDerivative(time);
  }
}

}  // namespace switched_model
