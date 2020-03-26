//
// Created by ruben on 14.09.18.
//

#ifndef OCS2_LOOPSHAPINGDYNAMICS_H
#define OCS2_LOOPSHAPINGDYNAMICS_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingDynamics : public ControlledSystemBase<FULL_STATE_DIM, FULL_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingDynamics>;

  using BASE = ControlledSystemBase<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_t;

  using SYSTEM = ocs2::ControlledSystemBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEM::state_vector_t;
  using system_input_vector_t = typename SYSTEM::input_vector_t;

  using filter_state_vector_t = Eigen::Matrix<scalar_t, FILTER_STATE_DIM, 1>;
  using filter_input_vector_t = Eigen::Matrix<scalar_t, FILTER_INPUT_DIM, 1>;

  ~LoopshapingDynamics() override = default;

  LoopshapingDynamics(const LoopshapingDynamics& obj)
      : BASE(), controlledSystem_(obj.controlledSystem_->clone()), loopshapingDefinition_(obj.loopshapingDefinition_) {}

  static std::unique_ptr<LoopshapingDynamics> create(const SYSTEM& controlledSystem,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void computeFlowMap(const scalar_t& time, const state_vector_t& state, const input_vector_t& input,
                      state_vector_t& stateDerivative) override {
    system_state_vector_t systemstate, systemstateDerivative;
    filter_state_vector_t filterstate, filterstateDerivative;
    system_input_vector_t systeminput;
    filter_input_vector_t filteredinput;

    loopshapingDefinition_->getSystemState(state, systemstate);
    loopshapingDefinition_->getSystemInput(state, input, systeminput);
    loopshapingDefinition_->getFilterState(state, filterstate);
    loopshapingDefinition_->getFilteredInput(state, input, filteredinput);

    controlledSystem_->computeFlowMap(time, systemstate, systeminput, systemstateDerivative);

    filterFlowmap(filterstate, filteredinput, systeminput, filterstateDerivative);

    loopshapingDefinition_->concatenateSystemAndFilterState(systemstateDerivative, filterstateDerivative, stateDerivative);
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    system_state_vector_t systemstate, systemMappedState;
    filter_state_vector_t filterstate, filterMappedState;
    loopshapingDefinition_->getSystemState(state, systemstate);
    loopshapingDefinition_->getFilterState(state, filterstate);

    controlledSystem_->computeJumpMap(time, systemstate, systemMappedState);

    // Filter doesn't Jump
    filterMappedState = filterstate;

    loopshapingDefinition_->concatenateSystemAndFilterState(systemMappedState, filterMappedState, mappedState);
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) override {
    system_state_vector_t systemstate;
    loopshapingDefinition_->getSystemState(state, systemstate);
    controlledSystem_->computeGuardSurfaces(time, systemstate, guardSurfacesValue);
  }

 protected:
  LoopshapingDynamics(const SYSTEM& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(), controlledSystem_(controlledSystem.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

 private:
  std::unique_ptr<SYSTEM> controlledSystem_;

  virtual void filterFlowmap(const filter_state_vector_t& x_filter, const filter_input_vector_t& u_filter,
                             const system_input_vector_t& u_system, filter_state_vector_t& filterStateDerivative) = 0;
};
};  // namespace ocs2

// Include classes derived from Loopshaping Dynamics to be dispatched by LoopshapingDynamics::Create
#include "LoopshapingDynamicsEliminatePattern.h"
#include "LoopshapingDynamicsInputPattern.h"
#include "LoopshapingDynamicsOutputPattern.h"

// Implement Factory method
namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
std::unique_ptr<LoopshapingDynamics<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>>
LoopshapingDynamics<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>::create(
    const SYSTEM& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingDynamics>(
          new LoopshapingDynamicsOutputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                               FILTER_INPUT_DIM>(controlledSystem, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingDynamics>(
          new LoopshapingDynamicsInputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                              FILTER_INPUT_DIM>(controlledSystem, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingDynamics>(
          new LoopshapingDynamicsEliminatePattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                  FILTER_INPUT_DIM>(controlledSystem, std::move(loopshapingDefinition)));
  }
};
};  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGDYNAMICS_H
