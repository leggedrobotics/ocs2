//
// Created by rgrandia on 25.09.20.
//

#include "TestAnymalSwitchedModel.h"

#include <ocs2_anymal_models/AnymalModels.h>

using namespace anymal;

class AnymalCrocSwitchedModelTests : public switched_model::TestAnymalSwitchedModel {
 public:
  AnymalCrocSwitchedModelTests()
      : TestAnymalSwitchedModel(getAnymalKinematics(AnymalModel::Croc), getAnymalKinematicsAd(AnymalModel::Croc),
                                getAnymalComModel(AnymalModel::Croc), getAnymalComModelAd(AnymalModel::Croc)) {}
};

TEST_F(AnymalCrocSwitchedModelTests, Constraints) {
  this->testConstraints();
}

TEST_F(AnymalCrocSwitchedModelTests, Kinematics) {
  this->printKinematics();
}

TEST_F(AnymalCrocSwitchedModelTests, ComDynamics) {
  this->printComModel();
}

TEST_F(AnymalCrocSwitchedModelTests, EndeffectorOrientation) {
  this->testEndeffectorOrientation();
}

TEST_F(AnymalCrocSwitchedModelTests, EndeffectorAlignedYAxisRandomHFEKFE) {
  this->testEndeffectorAlignedYAxisRandomHFEKFE();
}

TEST_F(AnymalCrocSwitchedModelTests, EndeffectorAlignedXAxisRandomHAA) {
  this->testEndeffectorAlignedXAxisRandomHAA();
}