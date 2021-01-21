//
// Created by rgrandia on 21.01.21.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

struct SwingNode {
  scalar_t time;
  scalar_t position;
  scalar_t velocity;
};

/**
 * Implements a quintic polynomial:
 *  y = c_5 * tau^5 + ... + c_1 * tau + c_0,
 *  where tau = (t - t0) / dt
 */
class QuinticSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuinticSpline() : c_(vector6_t::Zero()), t0_(0.0), dt_(0.0) {}
  QuinticSpline(vector6_t coefficients, scalar_t t0, scalar_t dt) : c_(coefficients), t0_(t0), dt_(dt) {}

  /** returns y(t) */
  scalar_t position(scalar_t time) const;

  /** returns dy/dt(t) */
  scalar_t velocity(scalar_t time) const;

  /** returns d2y/dt2(t) */
  scalar_t acceleration(scalar_t time) const;

 private:
  scalar_t normalizedTime(scalar_t t) const;

  /// Coefficients [c_5, ..., c_0] for normalized time
  vector6_t c_;
  scalar_t t0_;
  scalar_t dt_;
};

/**
 * Implements a swing trajectory based on two quintic splines
 * The trajectory is made with the following 12 conditions
 *  start:
 *      position = given
 *      velocity = given
 *      acceleration = 0.0
 *  Mid:
 *      position = given & continuous
 *      velocity = given & continuous
 *      acceleration = continuous
 *      jerk = continuous
 *  End:
 *      position = given
 *      velocity = given
 *      acceleration = 0.0
 */
class QuinticSwing {
 public:
  QuinticSwing(SwingNode start, scalar_t midHeight, SwingNode end);
  QuinticSwing(SwingNode start, SwingNode mid, SwingNode end);

  /** returns z(t) */
  scalar_t position(scalar_t time) const;

  /** returns dz/dt(t) */
  scalar_t velocity(scalar_t time) const;

  /** returns d2z/dt2(t) */
  scalar_t acceleration(scalar_t time) const;

 private:
  scalar_t midTime_;
  QuinticSpline leftSpline_;
  QuinticSpline rightSpline_;
};

}  // namespace switched_model