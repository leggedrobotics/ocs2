//
// Created by rgrandia on 29.04.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/core/Rotations.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

using namespace switched_model;

TerrainPlane getRandomTerrain() {
  vector3_t eulerXYZ = vector3_t::Random();
  return {vector3_t::Random(), rotationMatrixBaseToOrigin(eulerXYZ)};
}

std::vector<vector2_t> getRandomboundary(size_t N) {
  std::vector<vector2_t> boundary(N);
  for (auto& b : boundary) {
    b.setRandom();
  }
  return boundary;
}

TEST(TestConvexTerrain, projectToConvex2dPolygonBoundary_signTest) {
  constexpr size_t N = 3;
  const auto boundary = getRandomboundary(N);

  for (size_t numTests = 0; numTests < 3; numTests++) {
    // a point inside
    const auto insidePoint = [&]() -> vector2_t {
      vector_t lambda = 0.5 * vector_t::Random(N) + 0.5 * vector_t::Ones(N);  // positive numbers in [0, 1]
      lambda /= lambda.sum();

      vector2_t p = vector2_t::Zero();
      for (size_t i = 0; i < boundary.size(); i++) {
        p += lambda(i) * boundary[i];
      }
      return p;
    }();

    // a point outside
    const auto outsidePoint = [&]() -> vector2_t {
      const vector_t lambda = vector_t::Random(N) + 2.0 * vector_t::Ones(N);  // numbers in [1, 3]

      vector2_t p = vector2_t::Zero();
      for (size_t i = 0; i < boundary.size(); i++) {
        p += lambda(i) * boundary[i];
      }
      return p;
    }();

    // a point on the polygon boundary
    const auto onPoint = [&]() -> vector2_t {
      vector_t lambda = 0.5 * vector_t::Random(2) + 0.5 * vector_t::Ones(2);  // positive numbers in [0, 1]
      lambda /= lambda.sum();
      return lambda(0) * boundary[0] + lambda(1) * boundary[1];
    }();

    EXPECT_NEAR(projectToConvex2dPolygonBoundary(boundary, onPoint).first, 0.0, 1e-9);
    EXPECT_LT(projectToConvex2dPolygonBoundary(boundary, insidePoint).first, 0.0);
    EXPECT_GT(projectToConvex2dPolygonBoundary(boundary, outsidePoint).first, 0.0);
  }
}

TEST(TestConvexTerrain, projectToConvex2dPolygonBoundary_valueTest) {
  const std::vector<vector2_t> boundary{vector2_t(-1.0, -1.0), vector2_t(-1.0, 1.0), vector2_t(1.0, 1.0), vector2_t(1.0, -1.0)};

  for (size_t numTests = 0; numTests < 3; numTests++) {
    // a point inside
    const vector2_t insidePoint = vector2_t::Random();

    const scalar_array_t insidePointDists{-std::abs(insidePoint.x() - 1.0), -std::abs(insidePoint.x() + 1.0),
                                          -std::abs(insidePoint.y() - 1.0), -std::abs(insidePoint.y() + 1.0)};
    const auto minDist = std::max_element(insidePointDists.cbegin(), insidePointDists.cend());

    const auto distance2ImagePair = projectToConvex2dPolygonBoundary(boundary, insidePoint);
    const auto computedDist = (distance2ImagePair.first > 0.0) ? std::sqrt(distance2ImagePair.first) : -std::sqrt(-distance2ImagePair.first);

    EXPECT_NEAR(computedDist, *minDist, 1e-9);
  }
}

TEST(TestConvexTerrain, projectToConvexPolygon) {
  ConvexTerrain convexTerrain;
  convexTerrain.plane = getRandomTerrain();
  convexTerrain.boundary = {vector2_t(-0.5, -0.5), vector2_t(-0.5, 0.5), vector2_t(0.5, 0.5), vector2_t(0.5, -0.5)};

  // plane center point
  {
    const vector3_t point = convexTerrain.plane.positionInWorld;
    const vector3_t image = projectToConvex3dPolygon(convexTerrain, point);
    const vector3_t local_image = positionInTerrainFrameFromPositionInWorld(image, convexTerrain.plane);

    EXPECT_NEAR(local_image.x(), 0.0, 1e-9);
    EXPECT_NEAR(local_image.y(), 0.0, 1e-9);
    EXPECT_NEAR(local_image.z(), 0.0, 1e-9);
  }

  // random point
  {
    const vector3_t point = vector3_t::Random();
    const vector3_t image = projectToConvex3dPolygon(convexTerrain, point);
    const vector3_t local_image = positionInTerrainFrameFromPositionInWorld(image, convexTerrain.plane);

    EXPECT_LT(std::abs(local_image.x()), 0.5 + 1e-9);
    EXPECT_LT(std::abs(local_image.y()), 0.5 + 1e-9);
    EXPECT_NEAR(local_image.z(), 0.0, 1e-9);
  }
}
