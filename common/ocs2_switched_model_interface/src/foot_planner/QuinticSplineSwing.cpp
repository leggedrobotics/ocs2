//
// Created by rgrandia on 21.01.21.
//

#include "ocs2_switched_model_interface/foot_planner/QuinticSplineSwing.h"

namespace switched_model {

scalar_t QuinticSpline::position(scalar_t time) const {
  const scalar_t tau1 = normalizedTime(time);
  const scalar_t tau2 = tau1 * tau1;
  const scalar_t tau3 = tau2 * tau1;
  const scalar_t tau4 = tau3 * tau1;
  const scalar_t tau5 = tau4 * tau1;
  return c_[0] * tau5 + c_[1] * tau4 + c_[2] * tau3 + c_[3] * tau2 + c_[4] * tau1 + c_[5];
}

scalar_t QuinticSpline::velocity(scalar_t time) const {
  const scalar_t tau1 = normalizedTime(time);
  const scalar_t tau2 = tau1 * tau1;
  const scalar_t tau3 = tau2 * tau1;
  const scalar_t tau4 = tau3 * tau1;
  return (c_[0] * 5.0 * tau4 + c_[1] * 4.0 * tau3 + c_[2] * 3.0 * tau2 + c_[3] * 2.0 * tau1 + c_[4]) / dt_;
}

scalar_t QuinticSpline::acceleration(scalar_t time) const {
  const scalar_t tau1 = normalizedTime(time);
  const scalar_t tau2 = tau1 * tau1;
  const scalar_t tau3 = tau2 * tau1;
  return (c_[0] * 20.0 * tau3 + c_[1] * 12.0 * tau2 + c_[2] * 6.0 * tau1 + c_[3] * 2.0) / (dt_ * dt_);
}

scalar_t QuinticSpline::normalizedTime(scalar_t t) const {
  return (t - t0_) / dt_;
}

QuinticSwing::QuinticSwing(const SwingNode& start, scalar_t midHeight, const SwingNode& end)
    : QuinticSwing(start, SwingNode{0.5 * (start.time + end.time), midHeight, 0.0}, end) {}

QuinticSwing::QuinticSwing(const SwingNode& start, const SwingNode& mid, const SwingNode& end) : midTime_(mid.time) {
  assert(start.time < mid.time);
  assert(mid.time < end.time);

  // Compute time normalization
  const scalar_t dt_lhs = mid.time - start.time;
  const scalar_t dt_rhs = end.time - mid.time;
  const scalar_t scaling = dt_lhs / dt_rhs;
  const scalar_t scaling2 = scaling * scaling;
  const scalar_t scaling3 = scaling2 * scaling;

  // Set up normalized spline conditions as a linear system
  const size_t numConditions = 12;
  matrix_t A(numConditions, numConditions);
  // clang-format off
  A << // Lhs spline
       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, // start position
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, // start velocity (normalized)
       0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, // start acceleration (normalized)
       1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, // end position
       5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, // end velocity (normalized)
       // Rhs spline
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, // start position
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, // start velocity (normalized)
       0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, // end position
       0, 0, 0, 0, 0, 0, 5, 4, 3, 2, 1, 0, // end velocity (normalized)
       0, 0, 0, 0, 0, 0, 20, 12, 6, 2, 0, 0, // end acceleration (normalized)
       // Continuity
       20, 12, 6, 2, 0, 0, 0, 0, 0, -2 * scaling2, 0, 0, // acceleration (normalized)
       60, 24, 6, 0, 0, 0, 0, 0, -6 * scaling3, 0, 0, 0; // jerk (normalized)
  // clang-format on
  vector_t b(numConditions);
  b << start.position, start.velocity * dt_lhs, 0.0, mid.position, mid.velocity * dt_lhs, mid.position, mid.velocity * dt_rhs, end.position,
      end.velocity * dt_rhs, 0.0, 0.0, 0.0;

  // Solve for both splines
  vector_t coefficients = A.lu().solve(b);
  leftSpline_ = QuinticSpline(coefficients.segment<6>(0), start.time, dt_lhs);
  rightSpline_ = QuinticSpline(coefficients.segment<6>(6), mid.time, dt_rhs);
}

scalar_t QuinticSwing::position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
}

scalar_t QuinticSwing::velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
}

scalar_t QuinticSwing::acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
}

}  // namespace switched_model