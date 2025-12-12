//
// Created by rgrandia on 24.06.20.
//

#pragma once

#include <ocs2_core/misc/Benchmark.h>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "SegmentedPlanesTerrainModel.h"
#include "rclcpp/rclcpp.hpp"

namespace switched_model {

class SegmentedPlanesTerrainModelRos {
 public:
  SegmentedPlanesTerrainModelRos(const rclcpp::Node::SharedPtr& node);

  ~SegmentedPlanesTerrainModelRos();

  /// Extract the latest terrain model. Resets internal model to a nullptr
  std::unique_ptr<SegmentedPlanesTerrainModel> getTerrainModel();

  void createSignedDistanceBetween(const Eigen::Vector3d& minCoordinates,
                                   const Eigen::Vector3d& maxCoordinates);

  void publish();
  static void toPointCloud(
      const SegmentedPlanesSignedDistanceField& segmentedPlanesSignedDistanceField,
      sensor_msgs::msg::PointCloud2& pointCloud, size_t decimation,
      const std::function<bool(float)>& condition);

 private:
  void callback(
      const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr&
          msg);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> getSignedDistanceRange(
      const grid_map::GridMap& gridMap, const std::string& elevationLayer);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::
      SharedPtr terrainSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      distanceFieldPublisher_;

  std::mutex updateMutex_;
  std::atomic_bool terrainUpdated_;
  std::unique_ptr<SegmentedPlanesTerrainModel> terrainPtr_;

  std::mutex updateCoordinatesMutex_;
  Eigen::Vector3d minCoordinates_;
  Eigen::Vector3d maxCoordinates_;
  bool externalCoordinatesGiven_;

  std::mutex pointCloudMutex_;
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointCloud2MsgPtr_;

  ocs2::benchmark::RepeatedTimer callbackTimer_;
};

}  // namespace switched_model
