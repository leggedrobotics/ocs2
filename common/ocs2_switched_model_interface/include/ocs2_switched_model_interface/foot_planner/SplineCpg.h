/*
 * SplineCPG.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#pragma once

#include "ocs2_switched_model_interface/foot_planner/CubicSpline.h"

namespace switched_model {

class SplineCpg {
 public:
  SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown);

  SplineCpg(CubicSpline::Node liftOff, CubicSpline::Node midNode, CubicSpline::Node touchDown);

  scalar_t position(scalar_t time) const;

  scalar_t velocity(scalar_t time) const;

  scalar_t acceleration(scalar_t time) const;

  scalar_t startTimeDerivative(scalar_t time) const;

  scalar_t finalTimeDerivative(scalar_t time) const;

 private:
  scalar_t midTime_;
  CubicSpline leftSpline_;
  CubicSpline rightSpline_;
};

}  // namespace switched_model
