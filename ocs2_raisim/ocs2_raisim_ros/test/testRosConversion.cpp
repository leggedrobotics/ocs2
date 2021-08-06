#include <gtest/gtest.h>

#include "../../../ocs2_raisim/ocs2_raisim_ros/include/ocs2_raisim_ros/RaisimHeightmapRosConverter.h"

TEST(ocs2_raisim_ros, HeightmapRosConversion) {
  const raisim::TerrainProperties terrainProperties;
  constexpr double centerX = 0.2;
  constexpr double centerY = -2.1;

  const raisim::HeightMap heightMap1(centerX, centerY, terrainProperties);
  auto gridMap1 = ocs2::RaisimHeightmapRosConverter::convertHeightmapToGridmap(heightMap1);
  auto heightMap2 = ocs2::RaisimHeightmapRosConverter::convertGridmapToHeightmap(gridMap1);
  auto gridMap2 = ocs2::RaisimHeightmapRosConverter::convertHeightmapToGridmap(*heightMap2);

  // Test heightMap1 against heightMap2
  constexpr double probeResolution = 0.01;
  for (double pos_x = centerX - terrainProperties.xSize / 2.0; pos_x < centerX + terrainProperties.xSize / 2.0; pos_x += probeResolution) {
    for (double pos_y = centerY - terrainProperties.ySize / 2.0; pos_y < centerY + terrainProperties.ySize / 2.0;
         pos_y += probeResolution) {
      ASSERT_NEAR(heightMap1.getHeight(pos_x, pos_y), heightMap2->getHeight(pos_x, pos_y), 1e-7);
    }
  }

  // Test gridMap1 against gridMap2
  ASSERT_TRUE(gridMap1->data[0].data == gridMap2->data[0].data);
}

int main(int argc, char** argv) {
  // required for ros::Time()
  ros::Time::init();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
