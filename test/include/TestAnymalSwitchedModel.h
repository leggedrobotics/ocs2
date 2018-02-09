/*!
* @file    TestAnymalSwitchedModel.h
* @author  Jan Carius
* @date    Nov, 2017
*/

#pragma once

// model
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamics.h>
#include <ocs2_anymal_switched_model/dynamics_derivative/AnymalComKinoDynamicsDerivative.h>

namespace anymal {

class TestAnymalSwitchedModel {

public:

  TestAnymalSwitchedModel() :
    stanceLegs_({{true,true,true,true}}),
    comKinoDynamics_(stanceLegs_),
    comKinoDynamicsDerivative_(stanceLegs_){

  }

  void init() {
    // nothing to do yet
  }

public:
  AnymalKinematics kinematics_;
  AnymalCom comDynamics_;


  std::array<bool,4> stanceLegs_;
  AnymalComKinoDynamics comKinoDynamics_;
  AnymalComKinoDynamicsDerivative comKinoDynamicsDerivative_;

};

} // namespace anymal
