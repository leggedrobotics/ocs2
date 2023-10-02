//
// Created by rgrandia on 30.04.20.
//

#include "ocs2_quadruped_interface/TerrainPlaneVisualizer.h"

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include "ocs2_switched_model_interface/terrain/PlaneFitting.h"

namespace switched_model {

TerrainPlaneVisualizer::TerrainPlaneVisualizer(ros::NodeHandle& nodeHandle) {
  terrainPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/localTerrain", 1, true);
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

TerrainPlaneVisualizerSynchronizedModule::TerrainPlaneVisualizerSynchronizedModule(const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                                                   ros::NodeHandle& nodeHandle)
    : swingTrajectoryPlanner_(&swingTrajectoryPlanner), planeVisualizer_(nodeHandle) {}

void TerrainPlaneVisualizerSynchronizedModule::postSolverRun(const ocs2::PrimalSolution& primalSolution) {
  std::vector<vector3_t> regressionPoints;
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto& footholds = swingTrajectoryPlanner_->getNominalFootholds(leg);
    if (!footholds.empty()) {
      regressionPoints.push_back(footholds.front().plane.positionInWorld);
    }
  }

  if (regressionPoints.size() >= 3) {
    const auto normalAndPosition = estimatePlane(regressionPoints);
    TerrainPlane plane(normalAndPosition.position, orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
    planeVisualizer_.update(primalSolution.timeTrajectory_.front(), plane);
  }
}

}  // namespace switched_model
