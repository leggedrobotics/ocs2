#pragma once

#include <ocs2_oc/oc_solver/ModeScheduleManager.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"

namespace switched_model {

/**
 * Manages the ModeSchedule for switched model.
 */
class SwitchedModelModeScheduleManager : public ocs2::ModeScheduleManager<STATE_DIM, INPUT_DIM> {
 public:
  using Base = ocs2::ModeScheduleManager<STATE_DIM, INPUT_DIM>;

  explicit SwitchedModelModeScheduleManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr);

  ~SwitchedModelModeScheduleManager() override = default;

  contact_flag_t getContactFlags(scalar_t time) const;

  std::shared_ptr<GaitSchedule> getGaitSchedule() { return gaitSchedulePtr_; };

 private:
  void preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory, ocs2::ModeSchedule& modeSchedule) override;

 private:
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
};

}  // namespace switched_model
