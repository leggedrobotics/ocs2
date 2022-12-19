//
// Created by rgrandia on 24.06.20.
//

#include "segmented_planes_terrain_model/SegmentedPlanesTerrainModelRos.h"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <convex_plane_decomposition_ros/MessageConversion.h>

#include <grid_map_filters_rsl/lookup.hpp>

namespace switched_model {

SegmentedPlanesTerrainModelRos::SegmentedPlanesTerrainModelRos(ros::NodeHandle& nodehandle)
    : terrainUpdated_(false),
      minCoordinates_(Eigen::Vector3d::Zero()),
      maxCoordinates_(Eigen::Vector3d::Zero()),
      externalCoordinatesGiven_(false) {
  terrainSubscriber_ =
      nodehandle.subscribe("/convex_plane_decomposition_ros/planar_terrain", 1, &SegmentedPlanesTerrainModelRos::callback, this);
  distanceFieldPublisher_ =
      nodehandle.advertise<sensor_msgs::PointCloud2>("/convex_plane_decomposition_ros/signed_distance_field", 1, true);
}

SegmentedPlanesTerrainModelRos::~SegmentedPlanesTerrainModelRos() {
  if (callbackTimer_.getNumTimedIntervals() > 0) {
    std::cout << "[SegmentedPlanesTerrainModelRos] Benchmarking terrain Callback\n"
              << "\tStatistics computed over " << callbackTimer_.getNumTimedIntervals() << " iterations. \n"
              << "\tAverage time [ms] " << callbackTimer_.getAverageInMilliseconds() << "\n"
              << "\tMaximum time [ms] " << callbackTimer_.getMaxIntervalInMilliseconds() << std::endl;
  }
}

std::unique_ptr<SegmentedPlanesTerrainModel> SegmentedPlanesTerrainModelRos::getTerrainModel() {
  std::lock_guard<std::mutex> lock(updateMutex_);
  return std::move(terrainPtr_);
}

void SegmentedPlanesTerrainModelRos::createSignedDistanceBetween(const Eigen::Vector3d& minCoordinates,
                                                                 const Eigen::Vector3d& maxCoordinates) {
  std::lock_guard<std::mutex> lock(updateCoordinatesMutex_);
  minCoordinates_ = minCoordinates;
  maxCoordinates_ = maxCoordinates;
  externalCoordinatesGiven_ = true;
}

void SegmentedPlanesTerrainModelRos::publish() {
  // Extract point cloud.
  std::unique_ptr<sensor_msgs::PointCloud2> pointCloud2MsgPtr;
  {
    std::lock_guard<std::mutex> lock(pointCloudMutex_);
    pointCloud2MsgPtr.swap(pointCloud2MsgPtr_);
  }

  // Publish.
  if (pointCloud2MsgPtr != nullptr) {
    distanceFieldPublisher_.publish(*pointCloud2MsgPtr);
  }
}

void SegmentedPlanesTerrainModelRos::callback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
  callbackTimer_.startTimer();

  // Read terrain
  auto terrainPtr = std::make_unique<SegmentedPlanesTerrainModel>(convex_plane_decomposition::fromMessage(*msg));

  // Create SDF
  const std::string elevationLayer = "elevation";
  if (terrainPtr->planarTerrain().gridMap.exists(elevationLayer)) {
    const auto sdfRange = getSignedDistanceRange(terrainPtr->planarTerrain().gridMap, elevationLayer);
    terrainPtr->createSignedDistanceBetween(sdfRange.first, sdfRange.second);
  }

  // Create pointcloud for visualization
  const auto* sdfPtr = terrainPtr->getSignedDistanceField();
  if (sdfPtr != nullptr) {
    const auto& sdf = sdfPtr->asGridmapSdf();
    std::unique_ptr<sensor_msgs::PointCloud2> pointCloud2MsgPtr(new sensor_msgs::PointCloud2());
    grid_map::GridMapRosConverter::toPointCloud(sdf, *pointCloud2MsgPtr, 1, [](float val) { return -0.05F <= val && val <= 0.0F; });
    std::lock_guard<std::mutex> lock(pointCloudMutex_);
    pointCloud2MsgPtr_.swap(pointCloud2MsgPtr);
  }

  {  // Move to storage under the lock
    std::lock_guard<std::mutex> lock(updateMutex_);
    terrainPtr_.swap(terrainPtr);
  }

  callbackTimer_.endTimer();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SegmentedPlanesTerrainModelRos::getSignedDistanceRange(const grid_map::GridMap& gridMap,
                                                                                                   const std::string& elevationLayer) {
  // Extract coordinates for signed distance field
  Eigen::Vector3d minCoordinates;
  Eigen::Vector3d maxCoordinates;
  bool externalRangeGiven;
  {
    std::lock_guard<std::mutex> lock(updateCoordinatesMutex_);
    minCoordinates = minCoordinates_;
    maxCoordinates = maxCoordinates_;
    externalRangeGiven = externalCoordinatesGiven_;
  }

  if (!externalRangeGiven) {
    // Read min-max from elevation map
    const float heightMargin = 0.1;
    const auto& elevationData = gridMap.get(elevationLayer);
    const float minValue = elevationData.minCoeffOfFinites() - heightMargin;
    const float maxValue = elevationData.maxCoeffOfFinites() + heightMargin;
    auto minXY = grid_map::lookup::projectToMapWithMargin(
        gridMap, grid_map::Position(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()));
    auto maxXY = grid_map::lookup::projectToMapWithMargin(
        gridMap, grid_map::Position(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()));
    minCoordinates = {minXY.x(), minXY.y(), minValue};
    maxCoordinates = {maxXY.x(), maxXY.y(), maxValue};
  };

  return {minCoordinates, maxCoordinates};
}

}  // namespace switched_model
