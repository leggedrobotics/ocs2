//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/CubicSpline.h"

namespace switched_model {

CubicSpline::CubicSpline(Node start, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  scalar_t dp = end.position - start.position;
  scalar_t dv = end.velocity - start.velocity;

  dc0_ = 0.0;
  dc1_ = start.velocity;
  dc2_ = -(3.0 * start.velocity + dv);
  dc3_ = (2.0 * start.velocity + dv);

  c0_ = dc0_ * dt_ + start.position;
  c1_ = dc1_ * dt_;
  c2_ = dc2_ * dt_ + 3.0 * dp;
  c3_ = dc3_ * dt_ - 2.0 * dp;
}

scalar_t CubicSpline::position(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}

scalar_t CubicSpline::velocity(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

scalar_t CubicSpline::acceleration(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

scalar_t CubicSpline::startTimeDerivative(scalar_t t) const {
  scalar_t tn = normalizedTime(t);
  scalar_t dCoff = -(dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  scalar_t dTn = -(t1_ - t) / (dt_ * dt_);
  return velocity(t) * dt_ * dTn + dCoff;
}

scalar_t CubicSpline::finalTimeDerivative(scalar_t t) const {
  scalar_t tn = normalizedTime(t);
  scalar_t dCoff = (dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  scalar_t dTn = -(t - t0_) / (dt_ * dt_);
  return velocity(t) * dt_ * dTn + dCoff;
}

scalar_t CubicSpline::normalizedTime(scalar_t t) const {
  assert(t >= t0_);
  assert(t <= t1_);
  return (t - t0_) / dt_;
}

}  // namespace switched_model
