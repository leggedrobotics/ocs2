//
// Created by rgrandia on 28.09.20.
//

#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

#include <segmented_planes_terrain_model/SegmentedPlanesTerrainModelRos.h>

namespace switched_model {

class TerrainReceiverSynchronizedModule : public ocs2::SolverSynchronizedModule {
 public:
  TerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel>& terrainModel, ros::NodeHandle& nodeHandle);
  ~TerrainReceiverSynchronizedModule() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override{};

 private:
  ocs2::Synchronized<TerrainModel>* terrainModelPtr_;
  std::unique_ptr<switched_model::SegmentedPlanesTerrainModelRos> segmentedPlanesRos_;
};

}  // namespace switched_model