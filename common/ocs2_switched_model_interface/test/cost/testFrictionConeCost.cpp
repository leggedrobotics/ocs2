//
// Created by rgrandia on 27.06.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/cost/FrictionConeCost.h"

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenalty.h>

using namespace switched_model;

TEST(TestFrictionConeCost, equivalanceToDenseVersion) {
  // Mock the modeScheduleManager
  SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
  modeScheduleManager.setModeSchedule({{},{ModeNumber::STANCE}});

  // Mock the precomputation
  SwitchedModelPreComputationMockup preComp;

  // evaluation point
  scalar_t t = 0.0;
  comkino_state_t x = switched_model::comkino_state_t::Random();
  comkino_input_t u = switched_model::comkino_input_t::Random();
  ocs2::TargetTrajectories targetTrajectories({t}, {x}, {u});

  std::unique_ptr<ocs2::PenaltyBase> penalty(new ocs2::RelaxedBarrierPenalty({0.1, 5.0}));

  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    preComp.surfaceNormalInOriginFrame(i) = vector3_t(0.0, 0.0, 1.0);

    FrictionConeConstraint::Config frictionConfig;
    std::unique_ptr<FrictionConeConstraint> frictionCone(new FrictionConeConstraint(std::move(frictionConfig), i, modeScheduleManager));

    ocs2::StateInputSoftConstraint frictionConeCostDense(std::unique_ptr<FrictionConeConstraint>(frictionCone->clone()),
                                                         std::unique_ptr<ocs2::PenaltyBase>(penalty->clone()));

    FrictionConeCost frictionConeCostSparse(frictionConfig, i, modeScheduleManager, std::unique_ptr<ocs2::PenaltyBase>(penalty->clone()));

    const auto denseResult = frictionConeCostDense.getQuadraticApproximation(t, x, u, targetTrajectories, preComp);
    const auto sparseResult = frictionConeCostSparse.getQuadraticApproximation(t, x, u, targetTrajectories, preComp);

    scalar_t tol = 1e-12;
    EXPECT_DOUBLE_EQ(denseResult.f, sparseResult.f);
    EXPECT_LT((denseResult.dfdx - sparseResult.dfdx).norm(), tol);
    EXPECT_LT((denseResult.dfdu - sparseResult.dfdu).norm(), tol);
    EXPECT_LT((denseResult.dfdxx - sparseResult.dfdxx).norm(), tol);
    EXPECT_LT((denseResult.dfdux - sparseResult.dfdux).norm(), tol);
    EXPECT_LT((denseResult.dfduu - sparseResult.dfduu).norm(), tol);
  }
}