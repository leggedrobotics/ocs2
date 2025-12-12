/**
 * @authors     Fabian Jenelten
 * @affiliation RSL
 * @brief       Tests for grid map derivative filter.
 */

#include <gtest/gtest.h>

#include <grid_map_filters_rsl/GridMapDerivative.hpp>

using namespace grid_map;

TEST(TestGridMapDerivative, initialization) {  // NOLINT
  // Grid map with constant gradient.
  GridMap map;
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add("elevation", 1.0);
  const Eigen::MatrixXf H = map.get("elevation");
  GridMapIterator iterator(map);

  // Derivative filter.
  derivative::GridMapDerivative derivativeFilter;
  Eigen::Vector2d gradient;
  Eigen::Matrix2d curvature;

  // Compute derivatives for each element and check if constant.
  constexpr double threshold = 1.0e-9;
  for (; !iterator.isPastEnd(); ++iterator) {
    // Compute only gradient.
    derivativeFilter.estimateGradient(map, gradient, *iterator, H);
    EXPECT_TRUE(gradient.norm() < threshold);

    // Compute gradient and curvature.
    derivativeFilter.estimateGradientAndCurvature(map, gradient, curvature, *iterator, H);
    EXPECT_TRUE(gradient.norm() < threshold && curvature.norm() < threshold);
  }

  EXPECT_TRUE(iterator.isPastEnd());
}
