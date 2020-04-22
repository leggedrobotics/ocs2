//
// Created by rgrandia on 13.03.20.
//

#pragma once

#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"
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
                         const KinematicsModelBase<scalar_t>& kinematicsModel);

  void update(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState, const ocs2::ModeSchedule& modeSchedule,
              const TerrainPlane& terrain);

  void update(const ocs2::ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence);

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
  feet_array_t<std::vector<SplineCpg>> feetHeightTrajectories_;
  feet_array_t<std::vector<TerrainPlane>> targetTerrains_;  // TODO: consider merging into struct with SplineCpg
  feet_array_t<std::vector<scalar_t>> feetHeightTrajectoriesEvents_;

  // Error correction
  scalar_t initTime_;
  feet_array_t<scalar_t> initialErrors_;

 public:
  /** Helper functions */
  enum class FootPhaseType { Stance, Swing };
  struct FootPhase {
    FootPhaseType type;
    scalar_t startTime;  // times are NaN if they cannot be identified at the boundaries
    scalar_t endTime;
  };
  static std::vector<FootPhase> extractFootPhases(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

  /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
  static feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock);
};

}  // namespace switched_model
