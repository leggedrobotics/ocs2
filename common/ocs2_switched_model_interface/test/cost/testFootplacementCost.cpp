//
// Created by rgrandia on 27.06.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"

using namespace switched_model;

std::vector<vector2_t> createRegularPolygon(const vector2_t& center, double radius, int numberOfVertices) {
  assert(numberOfVertices > 2);
  std::vector<vector2_t> polygon;
  double angle = (2. * M_PI) / numberOfVertices;
  for (int i = 0; i < numberOfVertices; ++i) {
    double phi = i * angle;
    double px = radius * std::cos(phi) + center.x();
    double py = radius * std::sin(phi) + center.y();
    // Counter clockwise
    polygon.emplace_back(px, py);
  }
  return polygon;
}

TEST(TestFootplacementCost, constraintConversion) {
  const vector2_t center{1.0, 2.0};
  const scalar_t margin = 0.0;

  ConvexTerrain convexTerrain;
  convexTerrain.plane.positionInWorld = vector3_t{center.x(), center.y(), 3.0};
  convexTerrain.plane.orientationWorldToTerrain.setIdentity();

  const auto emptyConstraints = tangentialConstraintsFromConvexTerrain(convexTerrain, margin);
  ASSERT_EQ(emptyConstraints.A.rows(), 0);
  ASSERT_EQ(emptyConstraints.b.size(), 0);

  const int numVertices = 16;
  double radius = 1.0;
  convexTerrain.boundary = createRegularPolygon({0.0, 0.0}, 1.0, numVertices);
  const auto linConstraints = tangentialConstraintsFromConvexTerrain(convexTerrain, margin);
  ASSERT_EQ(linConstraints.A.rows(), numVertices);
  ASSERT_EQ(linConstraints.b.size(), numVertices);

  vector_t h = linConstraints.A * convexTerrain.plane.positionInWorld + linConstraints.b;
  vector_t h2 = linConstraints.A * vector3_t{center.x() + 0.99 * radius, center.y(), 0.0} + linConstraints.b;
  ASSERT_GT(h.minCoeff(), 0.0);
  ASSERT_GT(h2.minCoeff(), 0.0);
}