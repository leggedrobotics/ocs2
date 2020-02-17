//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"

namespace switched_model {

QuadrupedLoopshapingInterface::QuadrupedLoopshapingInterface(std::unique_ptr<QuadrupedInterface> quadrupedPtr,
                                                             const std::string& pathToConfigFolder)
    : quadrupedPtr_(std::move(quadrupedPtr)) {
  // Load some basic task settings
  scalar_t timeHorizon;
  size_t numPartitions;
  ocs2::loadData::loadPartitioningTimes(pathToConfigFolder + "/task.info", timeHorizon, numPartitions, partitioningTimes_, true);
  loadModeSequenceTemplate(pathToConfigFolder + "/task.info", "defaultModeSequenceTemplate", defaultModeSequenceTemplate_, true);

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
  operatingPointsPtr_.reset(new operating_point_t(quadrupedPtr_->getOperatingPoint(), loopshapingDefinition_));

  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, quadrupedPtr_->rolloutSettings()));
}

std::unique_ptr<QuadrupedLoopshapingInterface::slq_t> QuadrupedLoopshapingInterface::getSlq() const {
  return std::unique_ptr<slq_t>(new slq_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
                                          costFunctionPtr_.get(), operatingPointsPtr_.get(), quadrupedPtr_->slqSettings(),
                                          quadrupedPtr_->getLogicRulesPtr()));
}

std::unique_ptr<QuadrupedLoopshapingInterface::mpc_t> QuadrupedLoopshapingInterface::getMpc() const {
  if (!quadrupedPtr_->modelSettings().gaitOptimization_) {
    return std::unique_ptr<mpc_t>(new mpc_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
                                            costFunctionPtr_.get(), operatingPointsPtr_.get(), partitioningTimes_,
                                            quadrupedPtr_->slqSettings(), quadrupedPtr_->mpcSettings(), quadrupedPtr_->getLogicRulesPtr(),
                                            &defaultModeSequenceTemplate_));
  } else {
    throw std::runtime_error("mpc_ocs2 not configured, set gait optimization to 0");
  }
}

}  // namespace switched_model
