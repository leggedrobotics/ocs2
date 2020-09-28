//
// Created by rgrandia on 28.09.20.
//

#include "ocs2_quadruped_interface/TerrainReceiver.h"

namespace switched_model {

TerrainReceiverSynchronizedModule::TerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel>& terrainModel,
                                                                     ros::NodeHandle& nodeHandle)
    : terrainModelPtr_(&terrainModel), segmentedPlanesRos_(new switched_model::SegmentedPlanesTerrainModelRos(nodeHandle)) {}

void TerrainReceiverSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                                     const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  std::unique_ptr<switched_model::SegmentedPlanesTerrainModel> terrainUpdate;
  if (segmentedPlanesRos_->update(terrainUpdate)) {  // Fills the pointer if an update is available
    terrainModelPtr_->lock().reset(std::move(terrainUpdate));
  };
}

}  // namespace switched_model