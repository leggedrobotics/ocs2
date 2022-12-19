//
// Created by rgrandia on 18.02.22.
//

#include <ros/init.h>
#include <ros/master.h>
#include <ros/node_handle.h>

#include <geometry_msgs/PoseStamped.h>

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/foot_planner/KinematicFootPlacementPenalty.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace switched_model;

int main(int argc, char* argv[]) {
  // Initialize ros node
  ros::init(argc, argv, "anymal_mpc_visualize_foothold_selection");
  ros::NodeHandle nodeHandle;

  ApproximateKinematicsConfig config;
  config.maxLegExtension = 0.55;
  config.kinematicPenaltyWeight = 1.0;

  const vector3_t hipPositionInWorld(0.1, 0.2, 0.7);
  const vector3_t hipOrientation(0.3, 0.5, 1.3);
  const matrix3_t rotationHipToWorld = rotationMatrixBaseToOrigin(hipOrientation);

  grid_map::GridMap gridMap;
  gridMap.setGeometry({2.0, 2.0}, 0.04);
  gridMap.add("h0_height", 0.0);
  gridMap.add("h0_cost", 0.0);
  gridMap.add("h1_height", 0.2);
  gridMap.add("h1_cost", 0.0);
  gridMap.add("h2_height", 0.4);
  gridMap.add("h2_cost", 0.0);

  auto mapPublisher_ = nodeHandle.advertise<grid_map_msgs::GridMap>("map", 1);
  auto posePublisher_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("hipPose", 1);

  ros::WallRate rate(1.0);
  while (ros::ok()) {
    const auto time = ros::Time::now();

    auto& costLayer = gridMap.get("h0_cost");

    for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
      const auto index = *iterator;
      grid_map::Position position2D;
      gridMap.getPosition(index, position2D);

      for (const std::string prefix : {"h0", "h1", "h2"}) {
        vector3_t footPosition(position2D.x(), position2D.y(), gridMap.at(prefix + "_height", index));
        gridMap.at(prefix + "_cost", index) = computeKinematicPenalty(footPosition, hipPositionInWorld, rotationHipToWorld, config);
      }
    }

    geometry_msgs::PoseStamped hipPose;
    hipPose.header.frame_id = "world";
    hipPose.header.stamp = time;
    hipPose.pose.position = ocs2::getPointMsg(hipPositionInWorld);
    hipPose.pose.orientation = ocs2::getOrientationMsg(quaternionBaseToOrigin<double>(hipOrientation));
    posePublisher_.publish(hipPose);

    gridMap.setFrameId("world");
    gridMap.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap outputMessage;
    grid_map::GridMapRosConverter::toMessage(gridMap, outputMessage);
    mapPublisher_.publish(outputMessage);

    rate.sleep();
  }

  return 0;
}