/*
 * SplineCPG.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#pragma once

#include "ocs2_switched_model_interface/foot_planner/CubicSpline.h"

namespace switched_model {

struct SplineCpgSettings {
  using scalar_t = ocs2::Dimensions<0, 0>::scalar_t;
  scalar_t liftOffVelocity;
  scalar_t touchDownVelocity;
};

class SplineCpg {
  using scalar_t = ocs2::Dimensions<0, 0>::scalar_t;

 public:
  struct Point {
    scalar_t time;
    scalar_t height;
  };

  explicit SplineCpg(SplineCpgSettings settings);

  void set(Point liftoff, Point touchdown, scalar_t midHeight);

  scalar_t position(scalar_t time) const;

  scalar_t velocity(scalar_t time) const;

  scalar_t acceleration(scalar_t time) const;

  scalar_t startTimeDerivative(scalar_t time) const;

  scalar_t finalTimeDerivative(scalar_t time) const;

 private:
  SplineCpgSettings settings_;
  scalar_t midTime_;
  CubicSpline leftSpline_;
  CubicSpline rightSpline_;
};

}  // namespace switched_model
