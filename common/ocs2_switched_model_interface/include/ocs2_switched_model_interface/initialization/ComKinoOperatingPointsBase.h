#pragma once

#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class ComKinoOperatingPointsBase : public ocs2::SystemOperatingTrajectoriesBase {
 public:
  using Base = ocs2::SystemOperatingTrajectoriesBase;
  using com_model_t = ComModelBase<scalar_t>;

  ComKinoOperatingPointsBase(const com_model_t& comModel, std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr);

  ComKinoOperatingPointsBase(const ComKinoOperatingPointsBase& rhs);

  ~ComKinoOperatingPointsBase() override = default;

  ComKinoOperatingPointsBase* clone() const override;

  void getSystemOperatingTrajectories(const vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_array_t& timeTrajectory,
                                      vector_array_t& stateTrajectory, vector_array_t& inputTrajectory, bool concatOutput) override;

 private:
  input_vector_t computeInputOperatingPoints(const contact_flag_t& contactFlags, const state_vector_t& nominalState) const;

  std::unique_ptr<com_model_t> comModelPtr_;
  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
};

}  // end of namespace switched_model
