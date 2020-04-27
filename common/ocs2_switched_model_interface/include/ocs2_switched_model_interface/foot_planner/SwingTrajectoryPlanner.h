//
// Created by rgrandia on 13.03.20.
//

#pragma once

#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/terrain/TerrainModel.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

struct SwingTrajectoryPlannerSettings {
  scalar_t liftOffVelocity = 0.0;
  scalar_t touchDownVelocity = 0.0;
  scalar_t swingHeight = 0.1;
  scalar_t touchdownAfterHorizon = 0.2;  // swing time added beyond the horizon if there is no touchdown in the current mode schedule
  scalar_t errorGain = 0.0;        // proportional gain for returning to the planned swing trajectory. 10-90%-rise_time ~= 2.2 / errorGain
                                   // alternatively can be measured as (velocity feedback) / (tracking error) ([m/s] / [m])
  scalar_t swingTimeScale = 0.15;  // swing phases shorter than this time will be scaled down in height and velocity
};

SwingTrajectoryPlannerSettings loadSwingTrajectorySettings(const std::string& filename, bool verbose = true);

class SwingTrajectoryPlanner {
 public:
  SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const ComModelBase<scalar_t>& comModel,
                         const KinematicsModelBase<scalar_t>& kinematicsModel, std::shared_ptr<const TerrainModel> terrainModelPtr);

  void update(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState, const ocs2::ModeSchedule& modeSchedule,
              const TerrainPlane& terrain);

  const TerrainPlane& getReferenceTerrainPlane(size_t leg, scalar_t time) const;

  scalar_t getNormalDirectionVelocityConstraint(size_t leg, scalar_t time) const;

  scalar_t getNormalDirectionPositionConstraint(size_t leg, scalar_t time) const;

 private:
  void updateFeetTrajectories(scalar_t initTime, scalar_t finalTime, const feet_array_t<vector3_t>& currentFeetPositions,
                              const ocs2::ModeSchedule& modeSchedule, const TerrainPlane& terrain);

  void updateErrorTrajectories(scalar_t initTime, const feet_array_t<vector3_t>& currentFeetPositions,
                               const ocs2::ModeSchedule& modeSchedule, const TerrainPlane& terrain);

  SwingTrajectoryPlannerSettings settings_;
  std::unique_ptr<ComModelBase<scalar_t>> comModel_;
  std::unique_ptr<KinematicsModelBase<scalar_t>> kinematicsModel_;

  struct contactHistory {
    scalar_t time;
    vector3_t position;
  };
  feet_array_t<contactHistory> lastContacts_;

  feet_array_t<std::vector<FootPhase>> feetNormalTrajectories_;
  feet_array_t<std::vector<scalar_t>> feetNormalTrajectoriesEvents_;

  // Terrain
  std::shared_ptr<const TerrainModel> terrainModelPtr_;
};

/**
 * Get {startTime, endTime} for all contact phases. Swingphases are always implied in between: endTime[i] < startTime[i+1]
 * times are NaN if they cannot be identified at the boundaries
 */
std::vector<std::pair<scalar_t, scalar_t>> extractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);


/**
 * Extracts for each leg the contact sequence over the motion phase sequence.
 * @param modeSequence : Sequence of contact modes.
 * @return Sequence of contact flags per leg.
 */
feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& modeSequence);

}  // namespace switched_model
