//
// Created by rgrandia on 24.06.20.
//

#include "segmented_planes_terrain_model/SegmentedPlanesTerrainModelRos.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>

#include <grid_map_filters_rsl/lookup.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace switched_model {

SegmentedPlanesTerrainModelRos::SegmentedPlanesTerrainModelRos(
    const rclcpp::Node::SharedPtr& node)
    : node_(node),
      terrainUpdated_(false),
      minCoordinates_(Eigen::Vector3d::Zero()),
      maxCoordinates_(Eigen::Vector3d::Zero()),
      externalCoordinatesGiven_(false) {
  terrainSubscriber_ = node->create_subscription<
      convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      "/convex_plane_decomposition_ros/planar_terrain", 1,
      std::bind(&SegmentedPlanesTerrainModelRos::callback, this,
                std::placeholders::_1));
  distanceFieldPublisher_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/convex_plane_decomposition_ros/signed_distance_field", 1);
}

SegmentedPlanesTerrainModelRos::~SegmentedPlanesTerrainModelRos() {
  if (callbackTimer_.getNumTimedIntervals() > 0) {
    std::cout
        << "[SegmentedPlanesTerrainModelRos] Benchmarking terrain Callback\n"
        << "\tStatistics computed over "
        << callbackTimer_.getNumTimedIntervals() << " iterations. \n"
        << "\tAverage time [ms] " << callbackTimer_.getAverageInMilliseconds()
        << "\n"
        << "\tMaximum time [ms] "
        << callbackTimer_.getMaxIntervalInMilliseconds() << std::endl;
  }
}

std::unique_ptr<SegmentedPlanesTerrainModel>
SegmentedPlanesTerrainModelRos::getTerrainModel() {
  std::lock_guard<std::mutex> lock(updateMutex_);
  return std::move(terrainPtr_);
}

void SegmentedPlanesTerrainModelRos::createSignedDistanceBetween(
    const Eigen::Vector3d& minCoordinates,
    const Eigen::Vector3d& maxCoordinates) {
  std::lock_guard<std::mutex> lock(updateCoordinatesMutex_);
  minCoordinates_ = minCoordinates;
  maxCoordinates_ = maxCoordinates;
  externalCoordinatesGiven_ = true;
}

void SegmentedPlanesTerrainModelRos::publish() {
  // Extract point cloud.
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointCloud2MsgPtr;
  {
    std::lock_guard<std::mutex> lock(pointCloudMutex_);
    pointCloud2MsgPtr.swap(pointCloud2MsgPtr_);
  }

  // Publish.
  if (pointCloud2MsgPtr != nullptr) {
    distanceFieldPublisher_->publish(*pointCloud2MsgPtr);
  }
}

void SegmentedPlanesTerrainModelRos::callback(
    const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr&
        msg) {
  callbackTimer_.startTimer();

  // Read terrain
  auto terrainPtr = std::make_unique<SegmentedPlanesTerrainModel>(
      convex_plane_decomposition::fromMessage(*msg));

  // Create SDF
  const std::string elevationLayer = "elevation";
  if (terrainPtr->planarTerrain().gridMap.exists(elevationLayer)) {
    const auto sdfRange = getSignedDistanceRange(
        terrainPtr->planarTerrain().gridMap, elevationLayer);
    terrainPtr->createSignedDistanceBetween(sdfRange.first, sdfRange.second);
  }

  // Create pointcloud for visualization
  const auto* sdfPtr = terrainPtr->getSignedDistanceField();
  if (sdfPtr != nullptr) {
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointCloud2MsgPtr(
        new sensor_msgs::msg::PointCloud2());
    toPointCloud(*sdfPtr, *pointCloud2MsgPtr, 1,
                 [](float val) { return -0.05F <= val && val <= 0.0F; });
    std::lock_guard<std::mutex> lock(pointCloudMutex_);
    pointCloud2MsgPtr_.swap(pointCloud2MsgPtr);
  }

  {  // Move to storage under the lock
    std::lock_guard<std::mutex> lock(updateMutex_);
    terrainPtr_.swap(terrainPtr);
  }

  callbackTimer_.endTimer();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
SegmentedPlanesTerrainModelRos::getSignedDistanceRange(
    const grid_map::GridMap& gridMap, const std::string& elevationLayer) {
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
        gridMap, grid_map::Position(std::numeric_limits<double>::lowest(),
                                    std::numeric_limits<double>::lowest()));
    auto maxXY = grid_map::lookup::projectToMapWithMargin(
        gridMap, grid_map::Position(std::numeric_limits<double>::max(),
                                    std::numeric_limits<double>::max()));
    minCoordinates = {minXY.x(), minXY.y(), minValue};
    maxCoordinates = {maxXY.x(), maxXY.y(), maxValue};
  };

  return {minCoordinates, maxCoordinates};
}

void SegmentedPlanesTerrainModelRos::toPointCloud(
    const SegmentedPlanesSignedDistanceField&
        segmentedPlanesSignedDistanceField,
    sensor_msgs::msg::PointCloud2& pointCloud, size_t decimation,
    const std::function<bool(float)>& condition) {
  const auto& gridMap = segmentedPlanesSignedDistanceField.gridMap();
  pcl::PointCloud<pcl::PointXYZI> points;
  const grid_map::SignedDistanceField& signedDistanceField =
      segmentedPlanesSignedDistanceField.asGridmapSdf();
  signedDistanceField.convertToPointCloud(points);

  pointCloud.header.stamp = rclcpp::Time(static_cast<int64_t>(gridMap.getTimestamp()));
  pointCloud.header.frame_id = gridMap.getFrameId();

  // Fields: Store each point analogous to pcl::PointXYZI
  const std::vector<std::string> fieldNames{"x", "y", "z", "intensity"};
  pointCloud.fields.clear();
  pointCloud.fields.reserve(fieldNames.size());
  size_t offset = 0;
  for (const auto& name : fieldNames) {
    sensor_msgs::msg::PointField pointField;
    pointField.name = name;
    pointField.count = 1;
    pointField.datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointField.offset = offset;
    pointCloud.fields.push_back(pointField);
    offset += pointField.count * sizeof(float);
  }

  // Resize
  const size_t pointCloudMaxSize{points.size()};
  pointCloud.height = 1;
  pointCloud.width = pointCloudMaxSize;
  pointCloud.point_step = offset;
  pointCloud.row_step = pointCloud.width * pointCloud.point_step;
  pointCloud.data.resize(pointCloud.height * pointCloud.row_step);

  // Fill data
  size_t addedPoints = 0;
  std::vector<float> filterPoints;
  for (auto it = points.begin(); it != points.end(); it++) {
    const auto sdfValue = it->intensity;
    if (condition(sdfValue)) {
      filterPoints.push_back(it->x);
      filterPoints.push_back(it->y);
      filterPoints.push_back(it->z);
      filterPoints.push_back(sdfValue);
      ++addedPoints;
    }
  }

  if (addedPoints != pointCloudMaxSize) {
    pointCloud.width = addedPoints;
    pointCloud.row_step = pointCloud.width * pointCloud.point_step;
    const auto size = pointCloud.height * pointCloud.row_step;
    pointCloud.data.resize(size);
    memcpy(pointCloud.data.data(), filterPoints.data(), size);
  }
}

}  // namespace switched_model
