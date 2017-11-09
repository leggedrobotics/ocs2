/*!
* @file    TestAnymalSwitchedModel.hpp
* @author  Jan Carius
* @date    Nov, 2017
*/

#pragma once

// model
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalComDynamics.h>

namespace anymal {

class TestAnymalSwitchedModel {

public:

  void init() {
    // nothing to do yet
  }

public:
  AnymalKinematics kinematics_;
  AnymalComDynamics comDynamics_;

};

} // namespace anymal
