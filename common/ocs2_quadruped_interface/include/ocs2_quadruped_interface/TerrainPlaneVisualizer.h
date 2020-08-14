//
// Created by rgrandia on 30.04.20.
//

#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <ocs2_core/misc/Lockable.h>
#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

namespace switched_model {

class TerrainPlaneVisualizer {
 public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "world";  // Frame name all messages are published in
  double planeWidth_ = 1.5;
  double planeLength_ = 1.5;
  double planeThickness_ = 0.005;
  double planeAlpha_ = 0.5;

  explicit TerrainPlaneVisualizer(ros::NodeHandle& nodeHandle);

  void update(scalar_t time, const TerrainPlane& terrainPlane);

 private:
  ros::Publisher terrainPublisher_;
};

class TerrainPlaneVisualizerSynchronizedModule : public ocs2::SolverSynchronizedModule {
 public:
  TerrainPlaneVisualizerSynchronizedModule(ocs2::LockablePtr<TerrainModel>& terrainPtr, ros::NodeHandle& nodeHandle);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override{};

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override;

 private:
  ocs2::LockablePtr<TerrainModel>* terrainPptr_;
  TerrainPlaneVisualizer planeVisualizer_;
};

}  // namespace switched_model
