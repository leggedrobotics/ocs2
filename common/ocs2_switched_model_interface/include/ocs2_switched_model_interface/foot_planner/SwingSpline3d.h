//
// Created by rgrandia on 03.08.21.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

#include "ocs2_switched_model_interface/foot_planner/QuinticSplineSwing.h"

namespace switched_model {

struct SwingNode3d {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  scalar_t time;
  vector3_t position;
  vector3_t velocity;
};

class SwingSpline3d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Construct a swing trajectory with given boundary conditions
   *
   * @param start : starting time and boundary conditions
   * @param mid : waypoint time, position, and velocity. Time must be between start and end.
   * @param end : ending time and boundary conditions
   */
  SwingSpline3d(const SwingNode3d& start, const SwingNode3d& mid, const SwingNode3d& end);

  /**
   * Construct a swing trajectory through the provided nodes (at least 3)
   */
  explicit SwingSpline3d(const std::vector<SwingNode3d>& nodes);

  /** returns p(t) */
  vector3_t position(scalar_t time) const;

  /** returns dp/dt(t) */
  vector3_t velocity(scalar_t time) const;

  /** returns d2p/dt2(t) */
  vector3_t acceleration(scalar_t time) const;

  /** returns d3p/dt3(t) */
  vector3_t jerk(scalar_t time) const;

 private:
  QuinticSwing x_;
  QuinticSwing y_;
  QuinticSwing z_;
};

}  // namespace switched_model