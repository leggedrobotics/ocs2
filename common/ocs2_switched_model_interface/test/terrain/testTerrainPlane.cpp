//
// Created by rgrandia on 29.04.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

using namespace switched_model;

TerrainPlane getRandomTerrain() {
  vector3_t eulerXYZ = vector3_t::Random();
  return {vector3_t::Random(), rotationMatrixBaseToOrigin(eulerXYZ)};
}

TEST(TestTerrainPlane, surfaceNormal) {
  TerrainPlane canonicalPlane{vector3_t::Zero(), matrix3_t::Identity()};

  ASSERT_TRUE(surfaceNormalInWorld(canonicalPlane).isApprox(vector3_t{0.0, 0.0, 1.0}));

  vector3_t eulerXYZ{0.0, 0.0, 0.3};
  const auto yawRotation = rotationMatrixBaseToOrigin(eulerXYZ);

  TerrainPlane yawRotatedPlane{vector3_t::Zero(), yawRotation*matrix3_t::Identity()};

  ASSERT_TRUE(surfaceNormalInWorld(yawRotatedPlane).isApprox(vector3_t{0.0, 0.0, 1.0}));
}

TEST(TestTerrainPlane, tangentialBasisFromSurfaceNormal) {
  const auto randomPlane = getRandomTerrain();
  const auto surfaceNormal = surfaceNormalInWorld(randomPlane);

  const auto tangentialBasis = tangentialBasisFromSurfaceNormal(surfaceNormal);

  // Normalized
  ASSERT_DOUBLE_EQ(tangentialBasis.row(0).norm(), 1.0);
  ASSERT_DOUBLE_EQ(tangentialBasis.row(1).norm(), 1.0);

  // Ortogonal
  const double tol = 1e-9;
  ASSERT_LT(tangentialBasis.row(0).dot(surfaceNormal), tol);
  ASSERT_LT(tangentialBasis.row(1).dot(surfaceNormal), tol);
  ASSERT_LT(tangentialBasis.row(0).dot(tangentialBasis.row(1)), tol);
}

TEST(TestTerrainPlane, projectPositionInWorldOntoPlane) {
  const auto randomPlane = getRandomTerrain();

  // Origin of plane is projected to itself
  ASSERT_TRUE(projectPositionInWorldOntoPlane(randomPlane.positionInWorld, randomPlane).isApprox(randomPlane.positionInWorld));

  // Random point projected
  const auto projectedPosition = projectPositionInWorldOntoPlane(vector3_t::Random(), randomPlane);

  // Distance to plane
  ASSERT_DOUBLE_EQ(terrainDistanceFromPositionInWorld(projectedPosition, randomPlane), 0.0);

  // Double projection
  ASSERT_TRUE(projectPositionInWorldOntoPlane(projectedPosition, randomPlane).isApprox(projectedPosition));
}

TEST(TestTerrainPlane, projectPositionInWorldOntoPlaneAlongGravity_flatTerrain) {
  vector3_t eulerXYZ(0, 0, 0.3);
  TerrainPlane flatTerrain = {vector3_t::Random(), rotationMatrixOriginToBase(eulerXYZ)};

  // Origin of plane is projected to itself
  ASSERT_TRUE(projectPositionInWorldOntoPlaneAlongGravity(flatTerrain.positionInWorld, flatTerrain).isApprox(flatTerrain.positionInWorld));

  // Any point in the horizontal plane should stay  point projected
  const vector3_t queryPosition = {1.0, 1.0, 0.0};
  const auto projectedPosition = projectPositionInWorldOntoPlaneAlongGravity(queryPosition, flatTerrain);

  ASSERT_TRUE(queryPosition.isApprox(projectedPosition));
}

TEST(TestTerrainPlane, projectPositionInWorldOntoPlaneAlongGravity_randomTerrain) {
  const auto randomPlane = getRandomTerrain();

  // Origin of plane is projected to itself
  ASSERT_TRUE(projectPositionInWorldOntoPlaneAlongGravity(randomPlane.positionInWorld, randomPlane).isApprox(randomPlane.positionInWorld));

  // Random point projected
  const vector3_t queryPosition = vector3_t::Random();
  const auto projectedPosition = projectPositionInWorldOntoPlaneAlongGravity(queryPosition, randomPlane);

  // x, y position remained to same
  ASSERT_DOUBLE_EQ(projectedPosition.x(), queryPosition.x());
  ASSERT_DOUBLE_EQ(projectedPosition.y(), queryPosition.y());

  // Distance to plane
  const double tol = 1e-9;
  ASSERT_LT(terrainDistanceFromPositionInWorld(projectedPosition, randomPlane), tol);

  // Double projection
  ASSERT_TRUE(projectPositionInWorldOntoPlane(projectedPosition, randomPlane).isApprox(projectedPosition));
}