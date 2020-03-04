/*!
 * @file   TestAnymalWheelsSwitchedModel.h
 * @author Marko Bjelonic
 * @date   Nov 27, 2019
 */

#pragma once

// model
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsKinematics.h>
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h>

namespace anymal {

class TestAnymalWheelsSwitchedModel {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TestAnymalWheelsSwitchedModel() :
    stanceLegs_({{true,true,true,true}})
    {
  }

  void init() {
    // nothing to do yet
  }

public:
  AnymalWheelsKinematics kinematics_;
  AnymalWheelsCom comDynamics_;

  std::array<bool,4> stanceLegs_;

};

} // namespace anymal
