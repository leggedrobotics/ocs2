//
// Created by rgrandia on 08.06.22.
//

#include <gtest/gtest.h>

#include <boost/filesystem/path.hpp>

#include <algorithm>

#include "convex_plane_decomposition/ConvexRegionGrowing.h"
#include "convex_plane_decomposition/GeometryUtils.h"
#include "convex_plane_decomposition/LoadGridmapFromImage.h"
#include "convex_plane_decomposition/PlaneDecompositionPipeline.h"
#include "convex_plane_decomposition/SegmentedPlaneProjection.h"

using namespace convex_plane_decomposition;

namespace {

/**
 * Brute force version of getBestPlanarRegionAtPositionInWorld, searches through all candidates without shortcuts
 */
PlanarTerrainProjection getBestPlanarRegionAtPositionInWorldNaive(const Eigen::Vector3d& positionInWorld,
                                                                  const std::vector<PlanarRegion>& planarRegions,
                                                                  const std::function<double(const Eigen::Vector3d&)>& penaltyFunction) {
  // Do full project per region first
  std::vector<PlanarTerrainProjection> individualProjections;
  std::for_each(planarRegions.begin(), planarRegions.end(), [&](const PlanarRegion& planarRegion) {
    const Eigen::Vector3d positionInTerrainFrame = planarRegion.transformPlaneToWorld.inverse() * positionInWorld;

    PlanarTerrainProjection projection;
    projection.regionPtr = &planarRegion;
    projection.positionInTerrainFrame = projectToPlanarRegion({positionInTerrainFrame.x(), positionInTerrainFrame.y()}, planarRegion);
    projection.positionInWorld =
        positionInWorldFrameFromPosition2dInPlane(projection.positionInTerrainFrame, planarRegion.transformPlaneToWorld);
    projection.cost = (positionInWorld - projection.positionInWorld).squaredNorm() + penaltyFunction(projection.positionInWorld);
    individualProjections.push_back(projection);
  });

  // Find the minimum cost projection
  return *std::min_element(individualProjections.begin(), individualProjections.end(),
                           [](const PlanarTerrainProjection& lhs, const PlanarTerrainProjection& rhs) { return lhs.cost < rhs.cost; });
}
}  // namespace

TEST(TestConvexApproximation, runOnDemoMap) {
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

  // Run pipeline.
  PlaneDecompositionPipeline pipeline(config);
  pipeline.update(std::move(elevationMap), elevationLayer);
  const auto& planarTerrain = pipeline.getPlanarTerrain();

  // Query a range of points
  for (double x = -submapSize.x() / 2.0; x < submapSize.x() / 2.0; x +=  submapSize.x() / 4.0) {
    for (double y = -submapSize.y() / 2.0; y < submapSize.y() / 2.0; y +=  submapSize.y() / 4.0) {
      for (double z = -heightScale; z < heightScale; z += heightScale / 2.0) {
        Eigen::Vector3d query{x, y, z};
        auto penaltyFunction = [](const Eigen::Vector3d& projectedPoint) { return 0.1 * std::abs(projectedPoint.z()); };

        // Run projection and naive projection
        const auto projection = getBestPlanarRegionAtPositionInWorld(query, planarTerrain.planarRegions, penaltyFunction);
        const auto projectionCheck = getBestPlanarRegionAtPositionInWorldNaive(query, planarTerrain.planarRegions, penaltyFunction);

        // Check they are the same
        ASSERT_EQ(projection.regionPtr, projectionCheck.regionPtr);
        ASSERT_DOUBLE_EQ(projection.cost, projectionCheck.cost);
        ASSERT_DOUBLE_EQ(projection.positionInTerrainFrame.x(), projectionCheck.positionInTerrainFrame.x());
        ASSERT_DOUBLE_EQ(projection.positionInTerrainFrame.y(), projectionCheck.positionInTerrainFrame.y());
        ASSERT_TRUE(projection.positionInWorld.isApprox(projectionCheck.positionInWorld));

        // Check convex approximation with a range of settings
        for (int numberOfVertices = 3; numberOfVertices < 7; ++numberOfVertices) {
          for (double growthFactor = 1.01; growthFactor < 1.3; growthFactor += 0.1) {
            const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numberOfVertices, growthFactor);

            ASSERT_TRUE(std::all_of(convexRegion.vertices_begin(), convexRegion.vertices_end(),
                                    [&](const CgalPoint2d& p) { return isInside(p, projection.regionPtr->boundaryWithInset.boundary); }));
          }
        }
      }
    }
  }
}
