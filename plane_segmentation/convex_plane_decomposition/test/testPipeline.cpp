//
// Created by rgrandia on 16.03.22.
//

#include <gtest/gtest.h>

#include <boost/filesystem/path.hpp>

#include "convex_plane_decomposition/LoadGridmapFromImage.h"
#include "convex_plane_decomposition/PlaneDecompositionPipeline.h"

using namespace convex_plane_decomposition;

TEST(TestPipeline, runOnDemoMap) {
  // Config
  PlaneDecompositionPipeline::Config config;
  const auto resolution = config.preprocessingParameters.resolution;
  const std::string elevationLayer{"elevation_test"};
  const std::string frameId{"odom_test"};
  const Eigen::Array2d submapSize(3.0, 3.0);
  std::string file = "terrain.png";
  double heightScale = 1.25;

  // Terrain Loading
  boost::filesystem::path filePath(__FILE__);
  std::string folder = filePath.parent_path().generic_string() + std::string{"/data/"};
  const auto loadedMap = loadGridmapFromImage(folder + file, elevationLayer, frameId, resolution, heightScale);
  bool success = false;
  auto elevationMap = loadedMap.getSubmap(loadedMap.getPosition(), submapSize, success);
  ASSERT_TRUE(success);

  // Run
  PlaneDecompositionPipeline pipeline(config);
  ASSERT_NO_THROW(pipeline.update(std::move(elevationMap), elevationLayer));
}
