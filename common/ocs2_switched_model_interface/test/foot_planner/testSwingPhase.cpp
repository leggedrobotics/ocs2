//
// Created by rgrandia on 06.05.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"

using namespace switched_model;

TEST(TestSwingPhase, flatTerrainSwing) {
  TerrainPlane flatTerrain;
  const SwingPhase::SwingEvent liftOff{0.5, 0.2, &flatTerrain};
  const SwingPhase::SwingEvent touchdown{1.0, -0.2, &flatTerrain};
  const scalar_t midHeight = 0.1;

  SwingPhase swingPhase(liftOff, midHeight, touchdown);

  auto startConstraint = swingPhase.getFootNormalConstraintInWorldFrame(liftOff.time, 0.0);
  ASSERT_TRUE(startConstraint.velocityMatrix.isApprox(surfaceNormalInWorld((flatTerrain)).transpose()));
  ASSERT_DOUBLE_EQ(startConstraint.positionMatrix.norm(), 0.0);
  ASSERT_LT(std::abs(startConstraint.constant+liftOff.velocity), 1e-9);

  auto midConstraint = swingPhase.getFootNormalConstraintInWorldFrame(0.5*(liftOff.time+touchdown.time), 0.0);
  ASSERT_TRUE(midConstraint.velocityMatrix.isApprox(surfaceNormalInWorld((flatTerrain)).transpose()));
  ASSERT_DOUBLE_EQ(midConstraint.positionMatrix.norm(), 0.0);
  ASSERT_LT(std::abs(midConstraint.constant), 1e-9);

  auto endConstraint = swingPhase.getFootNormalConstraintInWorldFrame(touchdown.time, 0.0);
  ASSERT_TRUE(endConstraint.velocityMatrix.isApprox(surfaceNormalInWorld((flatTerrain)).transpose()));
  ASSERT_DOUBLE_EQ(endConstraint.positionMatrix.norm(), 0.0);
  ASSERT_LT(std::abs(endConstraint.constant+touchdown.velocity), 1e-9);
}