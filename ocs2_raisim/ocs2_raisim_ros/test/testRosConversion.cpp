/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <gtest/gtest.h>

#include "ocs2_raisim_ros/RaisimHeightmapRosConverter.h"

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
