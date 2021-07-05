//
// Created by rgrandia on 30.04.20.
//

#include "ocs2_quadruped_interface/TerrainPlaneVisualizer.h"

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

namespace switched_model {

TerrainPlaneVisualizer::TerrainPlaneVisualizer(ros::NodeHandle& nodeHandle) {
  terrainPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/localTerrain", 1);
}

void TerrainPlaneVisualizer::update(scalar_t time, const TerrainPlane& terrainPlane) {
  // Headers
  Eigen::Quaterniond terrainOrientation(terrainPlane.orientationWorldToTerrain.transpose());
  auto planeMsg =
      getPlaneMsg(terrainPlane.positionInWorld, terrainOrientation, ocs2::Color::black, planeWidth_, planeLength_, planeThickness_);
  planeMsg.header = ocs2::getHeaderMsg(frameId_, ros::Time::now());
  planeMsg.id = 0;
  planeMsg.color.a = planeAlpha_;

  terrainPublisher_.publish(planeMsg);
}

TerrainPlaneVisualizerSynchronizedModule::TerrainPlaneVisualizerSynchronizedModule(const ocs2::Synchronized<TerrainModel>& terrainModel,
                                                                                   ros::NodeHandle& nodeHandle)
    : terrainModelPtr_(&terrainModel), planeVisualizer_(nodeHandle) {}

void TerrainPlaneVisualizerSynchronizedModule::postSolverRun(const ocs2::PrimalSolution& primalSolution) {
  const base_coordinate_t comPose = getComPose(comkino_state_t(primalSolution.stateTrajectory_.front()));

  // Obtain local terrain below the base
  const auto localBaseTerrain = terrainModelPtr_->lock()->getLocalTerrainAtPositionInWorldAlongGravity(getPositionInOrigin(comPose));

  planeVisualizer_.update(primalSolution.timeTrajectory_.front(), localBaseTerrain);
}

}  // namespace switched_model
