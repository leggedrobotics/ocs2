/**
 * @authors     Fabian Jenelten, Ruben Grandia
 * @affiliation RSL
 * @brief       Tests for grid map lookup functions.
 */

#include <gtest/gtest.h>

#include <grid_map_filters_rsl/lookup.hpp>

using namespace grid_map;

TEST(TestLookup, maxValue_constant_map) {  // NOLINT
  // Grid map with constant value.
  GridMap map;
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  const float mapValue = 133.7;
  map.add("elevation", mapValue);
  const auto& data = map.get("elevation");

  {  // Normal lookup
    const grid_map::Position position1(-0.1, -0.2);
    const grid_map::Position position2(0.3, 0.4);
    const auto result = lookup::maxValueBetweenLocations(position1, position2, map, data);
    ASSERT_TRUE(result.isValid);
    EXPECT_DOUBLE_EQ(result.value, mapValue);
  }

  {  // Start and end are the same
    const grid_map::Position position1(-0.1, -0.2);
    const grid_map::Position position2 = position1;
    const auto result = lookup::maxValueBetweenLocations(position1, position2, map, data);
    ASSERT_TRUE(result.isValid);
    EXPECT_DOUBLE_EQ(result.value, mapValue);
  }

  {  // Start and end are outside of the map
    const grid_map::Position position1(1000.0, 1000.0);
    const grid_map::Position position2(2000.0, 2000.0);
    const auto result = lookup::maxValueBetweenLocations(position1, position2, map, data);
    ASSERT_TRUE(result.isValid);
    EXPECT_DOUBLE_EQ(result.value, mapValue);
  }
}

TEST(TestLookup, maxValue_in_middle_map) {  // NOLINT
  // Grid map with random
  GridMap map;
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  map.add("elevation", 0.0);
  auto& data = map.get("elevation");

  // Create a parabola on the map
  float checkMaxValue = std::numeric_limits<float>::lowest();
  for (int col = 0; col < data.cols(); ++col) {
    grid_map::Position p;
    map.getPosition({0, col}, p);
    float addedValue = -p.y() * p.y();
    checkMaxValue = std::max(checkMaxValue, addedValue);
    data.col(col).setConstant(-p.y() * p.y());
  }

  const grid_map::Position position1(-0.5, -0.5);
  const grid_map::Position position2(0.5, 0.5);
  const auto result = lookup::maxValueBetweenLocations(position1, position2, map, data);
  ASSERT_TRUE(result.isValid);
  EXPECT_DOUBLE_EQ(result.value, checkMaxValue);
}

TEST(TestLookup, maxValue_onlyNaN) {  // NOLINT
  // Grid map with constant value.
  GridMap map;
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  const float mapValue = std::numeric_limits<double>::quiet_NaN();
  map.add("elevation", mapValue);
  const auto& data = map.get("elevation");

  const grid_map::Position position1(-0.1, -0.2);
  const grid_map::Position position2(0.3, 0.4);
  const auto result = lookup::maxValueBetweenLocations(position1, position2, map, data);
  ASSERT_FALSE(result.isValid);
}
