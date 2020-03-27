#pragma once

#include <ocs2_core/initialization/SystemOperatingPoint.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class ComKinoOperatingPointsBase : public ocs2::SystemOperatingPoint<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ocs2::SystemOperatingPoint<STATE_DIM, INPUT_DIM>;
  using typename Base::input_vector_array_t;
  using typename Base::input_vector_t;
  using typename Base::state_vector_array_t;
  using typename Base::state_vector_t;

  using com_model_t = ComModelBase<scalar_t>;

  ComKinoOperatingPointsBase(const com_model_t& comModel, std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr);

  ComKinoOperatingPointsBase(const ComKinoOperatingPointsBase& rhs);

  ~ComKinoOperatingPointsBase() override = default;

  ComKinoOperatingPointsBase* clone() const override;

  void getSystemOperatingTrajectories(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime,
                                      scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory,
                                      input_vector_array_t& inputTrajectory, bool concatOutput = false) override;

 private:
  void computeInputOperatingPoints(contact_flag_t contactFlags, input_vector_t& inputs);

  std::unique_ptr<com_model_t> comModelPtr_;
  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
};

}  // end of namespace switched_model
