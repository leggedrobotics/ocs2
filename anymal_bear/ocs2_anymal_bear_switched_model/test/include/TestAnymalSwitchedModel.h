/*!
* @file    TestAnymalSwitchedModel.h
* @author  Jan Carius
* @date    Nov, 2017
*/

#pragma once

// model
#include <ocs2_anymal_bear_switched_model/core/AnymalBearCom.h>
#include <ocs2_anymal_bear_switched_model/core/AnymalBearKinematics.h>

namespace anymal {

class TestAnymalSwitchedModel {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TestAnymalSwitchedModel() :
    stanceLegs_({{true,true,true,true}})
    {
  }

  void init() {
    // nothing to do yet
  }

public:
  AnymalBearKinematics kinematics_;
  AnymalBearCom comDynamics_;

  std::array<bool,4> stanceLegs_;

};

} // namespace anymal
