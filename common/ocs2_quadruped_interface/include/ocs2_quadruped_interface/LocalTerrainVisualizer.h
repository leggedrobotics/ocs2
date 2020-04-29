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
  std::string originFrameId_ = "world";  // Frame name all messages are published in
  double planeWidth_ = 1.0;
  double planeLength_ = 2.0;
  double planeThickness_ = 0.005;
  double planeAlpha_ = 0.5;

  TerrainPlaneVisualizer(ros::NodeHandle& nodeHandle);

  void update(scalar_t time, const TerrainPlane& terrainPlane);

 private:
  ros::Publisher terrainPublisher_;
};

class LocalTerrainVisualizer : public ocs2::SolverSynchronizedModule<STATE_DIM, INPUT_DIM> {
 public:
  LocalTerrainVisualizer(ocs2::SharedLockablePPtr<TerrainModel> terrainPptr, ros::NodeHandle& nodeHandle);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override{};

  void postSolverRun(const primal_solution_t& primalSolution) override;

 private:
  ocs2::SharedLockablePPtr<TerrainModel> terrainPptr_;
  TerrainPlaneVisualizer planeVisualizer_;
};

}  // namespace switched_model
