//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"

namespace switched_model {

SplineCpg::SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown)
    : SplineCpg(liftOff, CubicSpline::Node{(liftOff.time + touchDown.time) / 2.0, midHeight, 0.0}, touchDown) {}

SplineCpg::SplineCpg(CubicSpline::Node liftOff, CubicSpline::Node midNode, CubicSpline::Node touchDown)
    : midTime_(midNode.time), leftSpline_(liftOff, midNode), rightSpline_(midNode, touchDown) {}

scalar_t SplineCpg::position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
}

scalar_t SplineCpg::velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
}

scalar_t SplineCpg::acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
}

scalar_t SplineCpg::startTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return leftSpline_.startTimeDerivative(time) + 0.5 * leftSpline_.startTimeDerivative(time);
  } else {
    return 0.5 * rightSpline_.startTimeDerivative(time);
  }
}

scalar_t SplineCpg::finalTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return 0.5 * leftSpline_.finalTimeDerivative(time);
  } else {
    return rightSpline_.finalTimeDerivative(time) + 0.5 * rightSpline_.finalTimeDerivative(time);
  }
}

}  // namespace switched_model
