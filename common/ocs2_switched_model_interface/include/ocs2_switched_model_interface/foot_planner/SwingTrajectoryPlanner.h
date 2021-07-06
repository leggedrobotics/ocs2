//
// Created by rgrandia on 13.03.20.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/logic/SingleLegLogic.h"
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
                         const KinematicsModelBase<scalar_t>& kinematicsModel);

  void update(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
              const ocs2::TargetTrajectories& targetTrajectories, const feet_array_t<std::vector<ContactTiming>>& contactTimingsPerLeg,
              const TerrainModel& terrainModel);

  const FootPhase& getFootPhase(size_t leg, scalar_t time) const;

  std::vector<TerrainPlane> getNominalFootholds(size_t leg) const { return nominalFootholdsPerLeg_[leg]; }

 private:
  void updateLastContact(int leg, scalar_t expectedLiftOff, const vector3_t& currentFootPosition, const TerrainModel& terrainModel);
  std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> generateSwingTrajectories(
      int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const;
  scalar_t getSwingMotionScaling(scalar_t liftoffTime, scalar_t touchDownTime) const;
  std::vector<TerrainPlane> selectNominalFootholdTerrain(int leg, const std::vector<ContactTiming>& contactTimings,
                                                         const ocs2::TargetTrajectories& targetTrajectories, scalar_t finalTime,
                                                         const TerrainModel& terrainModel) const;

  SwingTrajectoryPlannerSettings settings_;
  std::unique_ptr<ComModelBase<scalar_t>> comModel_;
  std::unique_ptr<KinematicsModelBase<scalar_t>> kinematicsModel_;

  feet_array_t<std::pair<scalar_t, TerrainPlane>> lastContacts_;
  feet_array_t<std::vector<std::unique_ptr<FootPhase>>> feetNormalTrajectories_;
  feet_array_t<std::vector<scalar_t>> feetNormalTrajectoriesEvents_;

  feet_array_t<std::vector<TerrainPlane>> nominalFootholdsPerLeg_;
};

}  // namespace switched_model
