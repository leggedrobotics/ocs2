#pragma once

#include <ocs2_core/misc/Lockable.h>
#include <ocs2_oc/oc_solver/ModeScheduleManager.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

/**
 * Manages the ModeSchedule for switched model.
 */
class SwitchedModelModeScheduleManager : public ocs2::ModeScheduleManager {
 public:
  using Base = ocs2::ModeScheduleManager;
  using LockableGaitSchedule = ocs2::Lockable<GaitSchedule>;
  using LockableTerrainModelPtr = ocs2::LockablePtr<TerrainModel>;

  SwitchedModelModeScheduleManager(GaitSchedule gaitSchedule, SwingTrajectoryPlanner swingTrajectory,
                                   std::unique_ptr<TerrainModel> terrainModel);

  ~SwitchedModelModeScheduleManager() override = default;

  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<LockableGaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

  LockableTerrainModelPtr& getTerrainPtr() { return terrainPtr_; }

 private:
  void preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory, ocs2::ModeSchedule& modeSchedule) override;

 private:
  std::shared_ptr<LockableGaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  LockableTerrainModelPtr terrainPtr_;
};

}  // namespace switched_model
