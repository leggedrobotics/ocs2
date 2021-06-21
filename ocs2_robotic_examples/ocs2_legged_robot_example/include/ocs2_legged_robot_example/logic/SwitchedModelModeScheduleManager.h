#pragma once

#include <ocs2_core/misc/Synchronized.h>

#include <ocs2_oc/synchronized_module/ModeScheduleManager.h>

#include <ocs2_legged_robot_example/common/definitions.h>
#include <ocs2_legged_robot_example/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_legged_robot_example/logic/GaitSchedule.h>
#include <ocs2_legged_robot_example/logic/MotionPhaseDefinition.h>

/**
 * Manages the ModeSchedule for switched model.
 */
namespace ocs2 {
namespace legged_robot {
class SwitchedModelModeScheduleManager : public ocs2::ModeScheduleManager {
 public:
  using Base = ocs2::ModeScheduleManager;

  SwitchedModelModeScheduleManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                   std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  SwitchedModelModeScheduleManager(const SwitchedModelModeScheduleManager& rhs);

  ~SwitchedModelModeScheduleManager() override = default;

  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 private:
  void preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory, ocs2::ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
