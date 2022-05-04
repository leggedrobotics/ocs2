//
// Created by rgrandia on 25.09.20.
//

#include "TestAnymalSwitchedModel.h"

#include <ocs2_anymal_models/AnymalModels.h>

using namespace anymal;

class AnymalWheelsChimeraSwitchedModelTests : public switched_model::TestAnymalSwitchedModel {
 public:
  AnymalWheelsChimeraSwitchedModelTests()
      : TestAnymalSwitchedModel(getAnymalKinematics(AnymalModel::WheelsChimera), getAnymalKinematicsAd(AnymalModel::WheelsChimera),
                                getAnymalComModel(AnymalModel::WheelsChimera), getAnymalComModelAd(AnymalModel::WheelsChimera)) {}
};

TEST_F(AnymalWheelsChimeraSwitchedModelTests, Cost) {
  this->testCosts();
}

TEST_F(AnymalWheelsChimeraSwitchedModelTests, Constraints) {
  this->testConstraints();
}

TEST_F(AnymalWheelsChimeraSwitchedModelTests, Kinematics) {
  this->printKinematics();
}

TEST_F(AnymalWheelsChimeraSwitchedModelTests, EndeffectorOrientation) {
  this->testEndeffectorOrientation();
}

