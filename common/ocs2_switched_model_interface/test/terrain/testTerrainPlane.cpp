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

  TerrainPlane yawRotatedPlane{vector3_t::Zero(), yawRotation * matrix3_t::Identity()};

  ASSERT_TRUE(surfaceNormalInWorld(yawRotatedPlane).isApprox(vector3_t{0.0, 0.0, 1.0}));
}

TEST(TestTerrainPlane, tangentialBasisFromSurfaceNormal) {
  const auto randomPlane = getRandomTerrain();
  const auto surfaceNormal = surfaceNormalInWorld(randomPlane);

  const auto tangentialBasis = tangentialBasisFromSurfaceNormal(surfaceNormal);

  // Normalized
  ASSERT_DOUBLE_EQ(tangentialBasis.row(0).norm(), 1.0);
  ASSERT_DOUBLE_EQ(tangentialBasis.row(1).norm(), 1.0);

  // Orthogonal
  const double tol = 1e-9;
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(1).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(tangentialBasis.row(1))), tol);
}

TEST(TestTerrainPlane, tangentialBasisFromSurfaceNormalUnitX) {
  const auto surfaceNormal = vector3_t::UnitX();
  const auto tangentialBasis = tangentialBasisFromSurfaceNormal(surfaceNormal);

  // Normalized
  ASSERT_DOUBLE_EQ(tangentialBasis.row(0).norm(), 1.0);
  ASSERT_DOUBLE_EQ(tangentialBasis.row(1).norm(), 1.0);

  // Orthogonal
  const double tol = 1e-9;
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(1).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(tangentialBasis.row(1))), tol);

  // Y is still unit y
  ASSERT_TRUE(tangentialBasis.row(1).transpose().isApprox(vector3_t::UnitY()));
}

TEST(TestTerrainPlane, orientationWorldToTerrainFromSurfaceNormalInWorld) {
  const vector3_t surfaceNormal = vector3_t::Random().normalized();

  // Extract
  const matrix3_t R_WtoT = orientationWorldToTerrainFromSurfaceNormalInWorld(surfaceNormal);
  const vector3_t xAxisInWorld = R_WtoT.row(0).transpose();
  const vector3_t yAxisInWorld = R_WtoT.row(1).transpose();
  const vector3_t zAxisInWorld = R_WtoT.row(2).transpose();

  const double tol = 1e-9;
  ASSERT_LT(std::abs(R_WtoT.determinant() - 1.0), tol);

  // z-axis is the surface normal
  ASSERT_TRUE(zAxisInWorld.isApprox(surfaceNormal));

  // Right hand coordinate system
  ASSERT_TRUE(zAxisInWorld.cross(xAxisInWorld).isApprox(yAxisInWorld));

  // Ortogonal
  ASSERT_LT(std::abs(xAxisInWorld.dot(yAxisInWorld)), tol);
  ASSERT_LT(std::abs(xAxisInWorld.dot(zAxisInWorld)), tol);
  ASSERT_LT(std::abs(yAxisInWorld.dot(zAxisInWorld)), tol);
}

TEST(TestTerrainPlane, orientationWorldToTerrainFromSurfaceNormalInWorld_identity) {
  // If the normal is in z direction in world. We want to retrieve the identity rotation. (local and world frame are aligned)
  const matrix3_t R_WtoT = orientationWorldToTerrainFromSurfaceNormalInWorld(vector3_t::UnitZ());
  ASSERT_TRUE(R_WtoT.isApprox(matrix3_t::Identity()));
}

TEST(TestTerrainPlane, projectPositionInWorldOntoPlane) {
  const auto randomPlane = getRandomTerrain();

  // Origin of plane is projected to itself
  ASSERT_TRUE(projectPositionInWorldOntoPlane(randomPlane.positionInWorld, randomPlane).isApprox(randomPlane.positionInWorld));

  // Random point projected
  const auto projectedPosition = projectPositionInWorldOntoPlane(vector3_t::Random(), randomPlane);

  // Distance to plane
  const double tol = 1e-9;
  ASSERT_LT(std::abs(terrainSignedDistanceFromPositionInWorld(projectedPosition, randomPlane)), tol);

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

  ASSERT_TRUE(queryPosition.head(2).isApprox(projectedPosition.head(2)));
  ASSERT_DOUBLE_EQ(projectedPosition.z(), flatTerrain.positionInWorld.z());
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
  ASSERT_LT(std::abs(terrainSignedDistanceFromPositionInWorld(projectedPosition, randomPlane)), tol);

  // Double projection
  ASSERT_TRUE(projectPositionInWorldOntoPlane(projectedPosition, randomPlane).isApprox(projectedPosition));
}