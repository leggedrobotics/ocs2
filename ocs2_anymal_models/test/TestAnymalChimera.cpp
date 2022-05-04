//
// Created by rgrandia on 25.09.20.
//

#include "TestAnymalSwitchedModel.h"

#include <ocs2_anymal_models/AnymalModels.h>

using namespace anymal;

class AnymalChimeraSwitchedModelTests : public switched_model::TestAnymalSwitchedModel {
 public:
  AnymalChimeraSwitchedModelTests()
      : TestAnymalSwitchedModel(getAnymalKinematics(AnymalModel::Chimera), getAnymalKinematicsAd(AnymalModel::Chimera),
                                getAnymalComModel(AnymalModel::Chimera), getAnymalComModelAd(AnymalModel::Chimera)) {}
};

TEST_F(AnymalChimeraSwitchedModelTests, Cost) {
  this->testCosts();
}

TEST_F(AnymalChimeraSwitchedModelTests, Constraints) {
  this->testConstraints();
}

TEST_F(AnymalChimeraSwitchedModelTests, Kinematics) {
  this->printKinematics();
}

TEST_F(AnymalChimeraSwitchedModelTests, EndeffectorOrientation) {
  this->testEndeffectorOrientation();
}
