//
// Created by ruben on 08.11.18.
//

#ifndef OCS2_LOOPSHAPINGOPERATINGPOINT_H
#define OCS2_LOOPSHAPINGOPERATINGPOINT_H

#include "ocs2_core/initialization/SystemOperatingTrajectoriesBase.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingOperatingPoint final : public SystemOperatingTrajectoriesBase<FULL_STATE_DIM, FULL_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingOperatingPoint>;

  using BASE = SystemOperatingTrajectoriesBase<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using SYSTEMBASE = SystemOperatingTrajectoriesBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEMBASE::state_vector_t;
  using system_state_vector_array_t = typename SYSTEMBASE::state_vector_array_t;
  using system_input_vector_t = typename SYSTEMBASE::input_vector_t;
  using system_input_vector_array_t = typename SYSTEMBASE::input_vector_array_t;

  using filter_state_vector_t = Eigen::Matrix<scalar_t, FILTER_STATE_DIM, 1>;
  using filter_input_vector_t = Eigen::Matrix<scalar_t, FILTER_INPUT_DIM, 1>;

  LoopshapingOperatingPoint(const SYSTEMBASE& systembase, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(), systembase_(systembase.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  virtual ~LoopshapingOperatingPoint() = default;

  LoopshapingOperatingPoint(const LoopshapingOperatingPoint& obj)
      : BASE(), systembase_(obj.systembase_->clone()), loopshapingDefinition_(obj.loopshapingDefinition_) {}

  LoopshapingOperatingPoint* clone() const override { return new LoopshapingOperatingPoint(*this); }

  void getSystemOperatingTrajectories(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime,
                                      scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory,
                                      input_vector_array_t& inputTrajectory, bool concatOutput = false) override {
    if (!concatOutput) {
      timeTrajectory.clear();
      stateTrajectory.clear();
      inputTrajectory.clear();
    }

    system_state_vector_t initialSystemState;
    scalar_array_t systemTimeTrajectory;
    system_state_vector_array_t systemStateTrajectory;
    system_input_vector_array_t systemInputTrajectory;
    loopshapingDefinition_->getSystemState(initialState, initialSystemState);

    // Get system operating point
    systembase_->getSystemOperatingTrajectories(initialSystemState, startTime, finalTime, systemTimeTrajectory, systemStateTrajectory,
                                                systemInputTrajectory, false);

    // Filter operating point
    filter_state_vector_t equilibriumFilterState;
    filter_input_vector_t equilibriumFilterInput;
    state_vector_t state;
    input_vector_t input;
    for (int k = 0; k < systemTimeTrajectory.size(); ++k) {
      loopshapingDefinition_->getFilterEquilibrium(systemInputTrajectory[k], equilibriumFilterState, equilibriumFilterInput);
      loopshapingDefinition_->concatenateSystemAndFilterInput(systemInputTrajectory[k], equilibriumFilterInput, input);
      loopshapingDefinition_->concatenateSystemAndFilterState(systemStateTrajectory[k], equilibriumFilterState, state);
      timeTrajectory.emplace_back(systemTimeTrajectory[k]);
      inputTrajectory.emplace_back(input);
      stateTrajectory.emplace_back(state);
    }
  }

 private:
  std::unique_ptr<SYSTEMBASE> systembase_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};
}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGOPERATINGPOINT_H
