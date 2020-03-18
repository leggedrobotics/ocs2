/*
 * CubicSpline.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_core/Dimensions.h>

namespace switched_model {

class CubicSpline {
  using scalar_t = ocs2::Dimensions<0, 0>::scalar_t;

 public:
  struct Node {
    scalar_t time;
    scalar_t position;
    scalar_t velocity;
  };

  CubicSpline();

  CubicSpline(Node start, Node end);

  scalar_t position(scalar_t time) const;

  scalar_t velocity(scalar_t time) const;

  scalar_t acceleration(scalar_t time) const;

  scalar_t startTimeDerivative(scalar_t t) const;

  scalar_t finalTimeDerivative(scalar_t t) const;

 private:
  scalar_t normalizedTime(scalar_t t) const;

  scalar_t t0_;
  scalar_t t1_;
  scalar_t dt_;

  scalar_t c0_;
  scalar_t c1_;
  scalar_t c2_;
  scalar_t c3_;

  scalar_t dc0_;  // derivative w.r.t. dt_
  scalar_t dc1_;  // derivative w.r.t. dt_
  scalar_t dc2_;  // derivative w.r.t. dt_
  scalar_t dc3_;  // derivative w.r.t. dt_
};

}  // namespace switched_model
