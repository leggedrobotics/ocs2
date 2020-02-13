//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingInterface.h"

#include <ocs2_anymal_bear_switched_model/core/AnymalCom.h>
#include <ocs2_anymal_bear_switched_model/core/AnymalKinematics.h>

namespace anymal {

AnymalBearLoopshapingInterface::AnymalBearLoopshapingInterface(const std::string& pathToConfigFolder)
    : BASE(AnymalKinematics(), AnymalCom(), pathToConfigFolder) {
  loopshapingDefinition_ = ocs2::loopshaping_property_tree::load(pathToConfigFolder + "/loopshaping.info");
  loopshapingDefinition_->print();
  filterDynamicsPtr_.reset(new filter_dynamics_t(loopshapingDefinition_));

  // cost function components
  Q_system_ = BASE::Q_.template block<SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>(0, 0);
  Q_system_final_ = BASE::QFinal_.template block<SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>(0, 0);
  R_system_ = BASE::R_.template block<SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM>(0, 0);

  // Initialize filter
  AnymalCom anymalCom;
  const auto totalWeight = anymalCom.totalMass() * 9.81;
  typename system_dynamics_t::system_input_vector_t uSystemForWeightCompensation;
  uSystemForWeightCompensation.setZero();
  size_t numLegs(4);
  for (size_t i = 0; i < numLegs; i++) {
    uSystemForWeightCompensation(3 * i + 2) = totalWeight / numLegs;
  }

  typename system_dynamics_t::filter_state_vector_t initialFilterState;
  typename system_dynamics_t::filter_input_vector_t initialFilterInput;
  loopshapingDefinition_->getFilterEquilibrium(uSystemForWeightCompensation, initialFilterState, initialFilterInput);

  // Also filter state here
  typename system_dynamics_t::system_state_vector_t initialSystemState;
  loopshapingDefinition_->getSystemState(initialState_, initialSystemState);
  loopshapingDefinition_->concatenateSystemAndFilterState(initialSystemState, initialFilterState, initialState_);

  // Set up normal anymal model
  anymalDynamicsPtr_.reset(new anymal_system_dynamics_t(AnymalKinematicsAd(), AnymalComAd(), modelSettings_.recompileLibraries_));
  anymalDynamicsDerivativesPtr_.reset(anymalDynamicsPtr_->clone());
  anymalConstraintsPtr_.reset(new anymal_constraint_t(AnymalKinematicsAd(), AnymalComAd(), logicRulesPtr_, modelSettings_));
  anymalCostFunctionPtr_.reset(new anymal_cost_function_t(AnymalCom(), logicRulesPtr_, Q_system_, R_system_, Q_system_final_));
  anymalOperatingPointPtr_.reset(new anymal_operating_point_t(AnymalCom(), logicRulesPtr_));

  // Wrap with loopshaping
  dynamicsPtr_ = system_dynamics_t::create(*anymalDynamicsPtr_, loopshapingDefinition_);
  dynamicsDerivativesPtr_ = system_dynamics_derivative_t::create(*anymalDynamicsDerivativesPtr_, loopshapingDefinition_);
  constraintsPtr_ = constraint_t::create(*anymalConstraintsPtr_, loopshapingDefinition_);
  costFunctionPtr_ = cost_function_t::create(*anymalCostFunctionPtr_, loopshapingDefinition_);
  operatingPointsPtr_.reset(new operating_point_t(*anymalOperatingPointPtr_, loopshapingDefinition_));

  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, BASE::rolloutSettings_));
}

std::unique_ptr<AnymalBearLoopshapingInterface::slq_t> AnymalBearLoopshapingInterface::getSlq() const {
  return std::unique_ptr<slq_t>(new slq_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
                                          costFunctionPtr_.get(), operatingPointsPtr_.get(), slqSettings_, logicRulesPtr_));
}

std::unique_ptr<AnymalBearLoopshapingInterface::mpc_t> AnymalBearLoopshapingInterface::getMpc() const {
  if (!modelSettings_.gaitOptimization_) {
    return std::unique_ptr<mpc_t>(new mpc_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
                                            costFunctionPtr_.get(), operatingPointsPtr_.get(), partitioningTimes_, slqSettings_,
                                            mpcSettings_, logicRulesPtr_, &defaultModeSequenceTemplate_));
  } else {
    throw std::runtime_error("mpc_ocs2 not configured, set gait optimization to 0");
  }
}

}  // end of namespace anymal
