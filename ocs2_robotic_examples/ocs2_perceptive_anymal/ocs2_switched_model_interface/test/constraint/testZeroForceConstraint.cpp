//
// Created by rgrandia on 19.09.19.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"
#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

using namespace switched_model;

TEST(TestZeroForceConstraint, evaluate) {
  // Mock the modeScheduleManager
  SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
  modeScheduleManager.setModeSchedule({{},{ModeNumber::FLY}});

  // Mock the precomputation
  SwitchedModelPreComputationMockup preComp;

  using TestedConstraint = ZeroForceConstraint;
  TestedConstraint zeroForceConstraint(0, modeScheduleManager);

  // evaluation point
  double t = 0.0;
  input_vector_t u;
  state_vector_t x;
  u.setZero();
  x.setZero();

  auto linearApproximation = zeroForceConstraint.getLinearApproximation(t, x, u, preComp);
  std::cout << "h: " << linearApproximation.f.transpose() << std::endl;
  std::cout << "dhdx: \n" << linearApproximation.dfdx << std::endl;
  std::cout << "dhdu: \n" << linearApproximation.dfdu << std::endl;
}
