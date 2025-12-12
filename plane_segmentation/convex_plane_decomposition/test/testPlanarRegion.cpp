//
// Created by rgrandia on 15.03.22.
//

#include <gtest/gtest.h>

#include "convex_plane_decomposition/PlanarRegion.h"

using namespace convex_plane_decomposition;

TEST(TestPlanarRegion, getTransformFromNormalAndPosition_identity) {
  NormalAndPosition normalAndPosition = {Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()};
  const auto transform = getTransformLocalToGlobal(normalAndPosition);

  // Orientation
  ASSERT_TRUE(transform.linear().isApprox(Eigen::Matrix3d::Identity()));

  // Position
  ASSERT_TRUE(transform.translation().isApprox(normalAndPosition.position));
}

TEST(TestPlanarRegion, getTransformFromNormalAndPosition_random) {
  NormalAndPosition normalAndPosition = {Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6).normalized()};
  const auto transform = getTransformLocalToGlobal(normalAndPosition);

  const Eigen::Vector3d xAxisInWorld = transform.linear().col(0);
  const Eigen::Vector3d yAxisInWorld = transform.linear().col(1);
  const Eigen::Vector3d zAxisInWorld = transform.linear().col(2);

  // Z direction
  ASSERT_TRUE(zAxisInWorld.isApprox(normalAndPosition.normal));

  // Orthogonal
  ASSERT_LT(std::abs(zAxisInWorld.dot(yAxisInWorld)), 1e-12);
  ASSERT_LT(std::abs(zAxisInWorld.dot(xAxisInWorld)), 1e-12);

  // Scale
  ASSERT_DOUBLE_EQ(xAxisInWorld.norm(), 1.0);
  ASSERT_DOUBLE_EQ(yAxisInWorld.norm(), 1.0);

  // Position
  ASSERT_TRUE(transform.translation().isApprox(normalAndPosition.position));
}

TEST(TestPlanarRegion, getTransformFromNormalAndPosition_unitX) {
  NormalAndPosition normalAndPosition = {Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d::UnitX()};
  const auto transform = getTransformLocalToGlobal(normalAndPosition);

  const Eigen::Vector3d xAxisInWorld = transform.linear().col(0);
  const Eigen::Vector3d yAxisInWorld = transform.linear().col(1);
  const Eigen::Vector3d zAxisInWorld = transform.linear().col(2);

  // Z direction
  ASSERT_TRUE(zAxisInWorld.isApprox(normalAndPosition.normal));

  // XY
  ASSERT_TRUE(xAxisInWorld.isApprox(-Eigen::Vector3d::UnitZ()));
  ASSERT_TRUE(yAxisInWorld.isApprox(Eigen::Vector3d::UnitY()));

  // Position
  ASSERT_TRUE(transform.translation().isApprox(normalAndPosition.position));
}

TEST(TestPlanarRegion, projectPositionInWorldOntoPlaneAlongGravity) {
  const NormalAndPosition normalAndPosition = {Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6).normalized()};
  const auto transformPlaneToWorld = getTransformLocalToGlobal(normalAndPosition);

  // Origin of plane is projected to 0, 0
  const CgalPoint2d originXY = {normalAndPosition.position.x(), normalAndPosition.position.y()};
  const CgalPoint2d originInPlane = projectToPlaneAlongGravity(originXY, transformPlaneToWorld);
  ASSERT_LT(std::abs(originInPlane.x()), 1e-12);
  ASSERT_LT(std::abs(originInPlane.y()), 1e-12);

  // Random point projected
  const Eigen::Vector2d queryPosition = Eigen::Vector2d::Random();
  const CgalPoint2d projectedPositionInPlane = projectToPlaneAlongGravity({queryPosition.x(), queryPosition.y()}, transformPlaneToWorld);

  // Convert back to world to check
  Eigen::Vector3d projectedPositionInPlane3d = {projectedPositionInPlane.x(), projectedPositionInPlane.y(), 0.0};
  Eigen::Vector3d projectedPositionInWorld3d = transformPlaneToWorld * projectedPositionInPlane3d;

  // x, y position remained to same
  ASSERT_DOUBLE_EQ(projectedPositionInWorld3d.x(), queryPosition.x());
  ASSERT_DOUBLE_EQ(projectedPositionInWorld3d.y(), queryPosition.y());
}