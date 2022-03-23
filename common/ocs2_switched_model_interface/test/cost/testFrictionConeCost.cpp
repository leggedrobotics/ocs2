//
// Created by rgrandia on 27.06.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/cost/FrictionConeCost.h"

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/penalties/Penalties.h>

using namespace switched_model;

TEST(TestFrictionConeCost, evaluate) {
  // Mock the modeScheduleManager
  SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
  modeScheduleManager.setModeSchedule({{}, {ModeNumber::STANCE}});

  // Mock the precomputation
  SwitchedModelPreComputationMockup preComp;

  // evaluation point
  scalar_t t = 0.0;
  comkino_state_t x = switched_model::comkino_state_t::Random();
  comkino_input_t u = switched_model::comkino_input_t::Random();
  ocs2::TargetTrajectories targetTrajectories({t}, {x}, {u});

  std::unique_ptr<ocs2::PenaltyBase> penalty(new ocs2::RelaxedBarrierPenalty({0.1, 5.0}));
  FrictionConeCost frictionConeCost(friction_cone::Config(), modeScheduleManager, std::move(penalty));

  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    preComp.surfaceNormalInOriginFrame(i) = vector3_t::Random();
    preComp.surfaceNormalInOriginFrame(i).normalize();
  }

  const auto cost = frictionConeCost.getValue(t, x, u, targetTrajectories, preComp);
  const auto costApproximation = frictionConeCost.getQuadraticApproximation(t, x, u, targetTrajectories, preComp);

  scalar_t tol = 1e-12;
  EXPECT_DOUBLE_EQ(cost, costApproximation.f);
  ASSERT_GT(ocs2::LinearAlgebra::symmetricEigenvalues(costApproximation.dfduu).minCoeff(), -tol);
  ASSERT_GT(ocs2::LinearAlgebra::symmetricEigenvalues(costApproximation.dfdxx).minCoeff(), -tol);
}

TEST(TestFrictionConeCost, finiteDifference) {
  // Mock the modeScheduleManager
  SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
  modeScheduleManager.setModeSchedule({{}, {ModeNumber::STANCE}});

  // Mock the precomputation
  SwitchedModelPreComputationMockup preComp;

  // evaluation point
  scalar_t t = 0.0;
  comkino_state_t x = switched_model::comkino_state_t::Random();
  comkino_input_t u = switched_model::comkino_input_t::Random();
  ocs2::TargetTrajectories targetTrajectories({t}, {x}, {u});

  std::unique_ptr<ocs2::PenaltyBase> penalty(new ocs2::RelaxedBarrierPenalty({0.1, 5.0}));
  FrictionConeCost frictionConeCost(friction_cone::Config(), modeScheduleManager, std::move(penalty));

  const scalar_t eps = 1e-4;
  const scalar_t tol = 1e-5;
  const int N = 10000;
  for (int i = 0; i < N; ++i) {
    comkino_state_t x = switched_model::comkino_state_t::Random();
    comkino_input_t u = switched_model::comkino_input_t::Random();

    comkino_state_t dx = eps * switched_model::comkino_state_t::Random();
    comkino_input_t du = eps * switched_model::comkino_input_t::Random();

    for (int leg = 0; leg< NUM_CONTACT_POINTS; ++leg) {
      preComp.surfaceNormalInOriginFrame(leg) = vector3_t::Random();
      preComp.surfaceNormalInOriginFrame(leg).normalize();
    }

    const auto cost = frictionConeCost.getValue(t, x, u, targetTrajectories, preComp);
    const auto costApproximation = frictionConeCost.getQuadraticApproximation(t, x, u, targetTrajectories, preComp);

    const auto cost_eps = frictionConeCost.getValue(t, x + dx, u + du, targetTrajectories, preComp);
    const auto cost_quadmodel = costApproximation.f + costApproximation.dfdx.dot(dx) + costApproximation.dfdu.dot(du) +
                                0.5 * dx.dot(costApproximation.dfdxx * dx) + 0.5 * du.dot(costApproximation.dfduu * du) +
                                du.dot(costApproximation.dfdux * dx);

    EXPECT_DOUBLE_EQ(cost, costApproximation.f);
    ASSERT_LT(std::abs(cost_eps - cost_quadmodel), tol);
  }
}
