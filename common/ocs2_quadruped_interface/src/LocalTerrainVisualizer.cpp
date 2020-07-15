//
// Created by rgrandia on 30.04.20.
//

#include "ocs2_quadruped_interface/LocalTerrainVisualizer.h"

#include "ocs2_quadruped_interface/QuadrupedVisualizationHelpers.h"

namespace switched_model {

TerrainPlaneVisualizer::TerrainPlaneVisualizer(ros::NodeHandle& nodeHandle) {
  terrainPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/localTerrain", 1);
}

void TerrainPlaneVisualizer::update(scalar_t time, const TerrainPlane& terrainPlane) {
  const auto timeStamp = ros::Time(time);

  // Headers
  Eigen::Quaterniond terrainOrientation(terrainPlane.orientationWorldToTerrain.transpose());
  auto planeMsg = getPlaneMsg(terrainPlane.positionInWorld, terrainOrientation, Color::black, planeWidth_, planeLength_, planeThickness_);
  planeMsg.header = getHeaderMsg(originFrameId_, timeStamp);
  planeMsg.id = 0;
  planeMsg.color.a = planeAlpha_;

  terrainPublisher_.publish(planeMsg);
}

LocalTerrainVisualizer::LocalTerrainVisualizer(ocs2::LockablePtr<TerrainModel>& terrainPtr, ros::NodeHandle& nodeHandle)
    : terrainPptr_(&terrainPtr), planeVisualizer_(nodeHandle) {}

void LocalTerrainVisualizer::postSolverRun(const ocs2::PrimalSolution& primalSolution) {
  const base_coordinate_t comPose = getComPose(comkino_state_t(primalSolution.stateTrajectory_.front()));

  // Obtain local terrain below the base
  const auto localBaseTerrain = [&] {
    std::lock_guard<ocs2::LockablePtr<TerrainModel>> lock(*terrainPptr_);
    return (*terrainPptr_)->getLocalTerrainAtPositionInWorld(getPositionInOrigin(comPose));
  }();

  planeVisualizer_.update(primalSolution.timeTrajectory_.front(), localBaseTerrain);
}

}  // namespace switched_model
