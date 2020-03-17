//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/CubicSpline.h"

namespace switched_model {

CubicSpline::CubicSpline() : CubicSpline({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}) {}

CubicSpline::CubicSpline(Node start, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  scalar_t dp = end.position - start.position;
  scalar_t dv = end.velocity - start.velocity;

  dev_c0_ = 0.0;
  dev_c1_ = start.velocity;
  dev_c2_ = -(3.0 * start.velocity + dv);
  dev_c3_ = (2.0 * start.velocity + dv);

  c0_ = dev_c0_ * dt_ + start.position;
  c1_ = dev_c1_ * dt_;
  c2_ = dev_c2_ * dt_ + 3.0 * dp;
  c3_ = dev_c3_ * dt_ - 2.0 * dp;
}

CubicSpline::scalar_t CubicSpline::position(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}

CubicSpline::scalar_t CubicSpline::velocity(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

CubicSpline::scalar_t CubicSpline::acceleration(scalar_t time) const {
  scalar_t tn = normalizedTime(time);
  return (6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

CubicSpline::scalar_t CubicSpline::startTimeDerivative(scalar_t t) const {
  scalar_t tn = normalizedTime(t);
  scalar_t dev_coff = -(dev_c3_ * tn * tn * tn + dev_c2_ * tn * tn + dev_c1_ * tn + dev_c0_);
  scalar_t dev_tn = -(t1_ - t) / (dt_ * dt_);
  return velocity(t) * dt_ * dev_tn + dev_coff;
}

CubicSpline::scalar_t CubicSpline::finalTimeDerivative(scalar_t t) const {
  scalar_t tn = normalizedTime(t);
  scalar_t dev_coff = (dev_c3_ * tn * tn * tn + dev_c2_ * tn * tn + dev_c1_ * tn + dev_c0_);
  scalar_t dev_tn = -(t - t0_) / (dt_ * dt_);
  return velocity(t) * dt_ * dev_tn + dev_coff;
}

CubicSpline::scalar_t CubicSpline::normalizedTime(scalar_t t) const {
  assert(t >= t0_);
  assert(t <= t1_);
  return (t - t0_) / dt_;
}

}  // namespace switched_model
