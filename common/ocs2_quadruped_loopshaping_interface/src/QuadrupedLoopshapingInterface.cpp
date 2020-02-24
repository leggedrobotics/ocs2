//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

QuadrupedLoopshapingInterface::QuadrupedLoopshapingInterface(std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr,
                                                             const std::string& pathToConfigFolder)
    : quadrupedPtr_(std::move(quadrupedPtr)) {
  // Load loopshaping
  loopshapingDefinition_ = ocs2::loopshaping_property_tree::load(pathToConfigFolder + "/loopshaping.info");
  loopshapingDefinition_->print();
  filterDynamicsPtr_.reset(new filter_dynamics_t(loopshapingDefinition_));

  // Initialize state including filter state
  const auto totalWeight = quadrupedPtr_->getComModel().totalMass() * 9.81;
  typename system_dynamics_t::system_input_vector_t uSystemForWeightCompensation;
  uSystemForWeightCompensation.setZero();
  size_t numLegs(4);
  for (size_t i = 0; i < numLegs; i++) {
    uSystemForWeightCompensation(3 * i + 2) = totalWeight / numLegs;
  }

  typename system_dynamics_t::filter_state_vector_t initialFilterState;
  typename system_dynamics_t::filter_input_vector_t initialFilterInput;
  loopshapingDefinition_->getFilterEquilibrium(uSystemForWeightCompensation, initialFilterState, initialFilterInput);
  loopshapingDefinition_->concatenateSystemAndFilterState(quadrupedPtr_->getInitialState(), initialFilterState, initialState_);

  // Wrap with loopshaping
  dynamicsPtr_ = system_dynamics_t::create(quadrupedPtr_->getDynamics(), loopshapingDefinition_);
  dynamicsDerivativesPtr_ = system_dynamics_derivative_t::create(quadrupedPtr_->getDynamicsDerivatives(), loopshapingDefinition_);
  constraintsPtr_ = constraint_t::create(*quadrupedPtr_->getConstraintPtr(), loopshapingDefinition_);
  costFunctionPtr_ = cost_function_t::create(quadrupedPtr_->getCost(), loopshapingDefinition_);
  operatingPointsPtr_.reset(new operating_point_t(quadrupedPtr_->getOperatingPoints(), loopshapingDefinition_));

  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, quadrupedPtr_->rolloutSettings()));
}

}  // namespace switched_model_loopshaping
