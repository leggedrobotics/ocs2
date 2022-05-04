//
// Created by rgrandia on 25.09.20.
//

#include "TestAnymalSwitchedModel.h"

#include <ocs2_anymal_models/AnymalModels.h>

using namespace anymal;

class AnymalWheelsSwitchedModelTests : public switched_model::TestAnymalSwitchedModel {
 public:
  AnymalWheelsSwitchedModelTests()
      : TestAnymalSwitchedModel(getAnymalKinematics(AnymalModel::Wheels), getAnymalKinematicsAd(AnymalModel::Wheels),
                                getAnymalComModel(AnymalModel::Wheels), getAnymalComModelAd(AnymalModel::Wheels)) {}
};

TEST_F(AnymalWheelsSwitchedModelTests, Cost) {
  this->testCosts();
}

TEST_F(AnymalWheelsSwitchedModelTests, Constraints) {
  this->testConstraints();
}

TEST_F(AnymalWheelsSwitchedModelTests, Kinematics) {
  this->printKinematics();
}

TEST_F(AnymalWheelsSwitchedModelTests, EndeffectorOrientation) {
  this->testEndeffectorOrientation();
}
