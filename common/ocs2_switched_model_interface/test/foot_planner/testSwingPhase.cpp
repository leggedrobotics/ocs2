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

  auto startConstraint = swingPhase.getFootNormalConstraintInWorldFrame(liftOff.time);
  ASSERT_TRUE(startConstraint.velocityMatrix.isApprox(surfaceNormalInWorld((flatTerrain)).transpose()));
  ASSERT_DOUBLE_EQ(startConstraint.positionMatrix.norm(), 0.0);
  ASSERT_LT(std::abs(startConstraint.constant + liftOff.velocity), 1e-9);

  auto midConstraint = swingPhase.getFootNormalConstraintInWorldFrame(0.5 * (liftOff.time + touchdown.time));
  ASSERT_TRUE(midConstraint.velocityMatrix.isApprox(surfaceNormalInWorld((flatTerrain)).transpose()));
  ASSERT_DOUBLE_EQ(midConstraint.positionMatrix.norm(), 0.0);
  ASSERT_LT(std::abs(midConstraint.constant), 1e-9);

  auto endConstraint = swingPhase.getFootNormalConstraintInWorldFrame(touchdown.time);
  ASSERT_TRUE(endConstraint.velocityMatrix.isApprox(surfaceNormalInWorld((flatTerrain)).transpose()));
  ASSERT_DOUBLE_EQ(endConstraint.positionMatrix.norm(), 0.0);
  ASSERT_LT(std::abs(endConstraint.constant + touchdown.velocity), 1e-9);
}

TEST(TestQuinticSwing, interpolating) {
  SwingNode start{0.5, 0.1, 0.2};
  SwingNode mid{0.75, 0.2, -0.3};
  SwingNode end{1.0, -0.4, 0.4};
  QuinticSwing quinticSwing(start, mid, end);

  double tol = 1e-9;
  EXPECT_LT(std::abs(quinticSwing.position(start.time) - start.position), tol);
  EXPECT_LT(std::abs(quinticSwing.velocity(start.time) - start.velocity), tol);
  EXPECT_LT(std::abs(quinticSwing.position(mid.time) - mid.position), tol);
  EXPECT_LT(std::abs(quinticSwing.velocity(mid.time) - mid.velocity), tol);
  EXPECT_LT(std::abs(quinticSwing.position(end.time) - end.position), tol);
  EXPECT_LT(std::abs(quinticSwing.velocity(end.time) - end.velocity), tol);
}

TEST(TestQuinticSwing3d, interpolating) {
  SwingNode3d start{0.5, vector3_t::Random(), vector3_t::Random()};
  SwingNode3d mid{0.75, vector3_t::Random(), vector3_t::Random()};
  SwingNode3d end{1.0, vector3_t::Random(), vector3_t::Random()};
  SwingSpline3d swingNode3D(start, mid, end);

  double tol = 1e-9;
  EXPECT_LT((swingNode3D.position(start.time) - start.position).norm(), tol);
  EXPECT_LT((swingNode3D.velocity(start.time) - start.velocity).norm(), tol);
  EXPECT_LT((swingNode3D.position(mid.time) - mid.position).norm(), tol);
  EXPECT_LT((swingNode3D.velocity(mid.time) - mid.velocity).norm(), tol);
  EXPECT_LT((swingNode3D.position(end.time) - end.position).norm(), tol);
  EXPECT_LT((swingNode3D.velocity(end.time) - end.velocity).norm(), tol);
}