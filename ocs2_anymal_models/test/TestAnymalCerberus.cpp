//
// Created by rgrandia on 25.09.20.
//

#include "TestAnymalSwitchedModel.h"

#include <ocs2_anymal_models/AnymalModels.h>

using namespace anymal;

class AnymalCerberusSwitchedModelTests : public switched_model::TestAnymalSwitchedModel {
 public:
  AnymalCerberusSwitchedModelTests()
      : TestAnymalSwitchedModel(getAnymalKinematics(AnymalModel::Cerberus), getAnymalKinematicsAd(AnymalModel::Cerberus),
                                getAnymalComModel(AnymalModel::Cerberus), getAnymalComModelAd(AnymalModel::Cerberus)) {}
};

TEST_F(AnymalCerberusSwitchedModelTests, Cost) {
  this->testCosts();
}

TEST_F(AnymalCerberusSwitchedModelTests, Constraints) {
  this->testConstraints();
}

TEST_F(AnymalCerberusSwitchedModelTests, Kinematics) {
  this->printKinematics();
}

TEST_F(AnymalCerberusSwitchedModelTests, EndeffectorOrientation) {
  this->testEndeffectorOrientation();
}

TEST_F(AnymalCerberusSwitchedModelTests, EndeffectorAlignedYAxisRandomHFEKFE) {
  this->testEndeffectorAlignedYAxisRandomHFEKFE();
}

TEST_F(AnymalCerberusSwitchedModelTests, EndeffectorAlignedXAxisRandomHAA) {
  this->testEndeffectorAlignedXAxisRandomHAA();
}