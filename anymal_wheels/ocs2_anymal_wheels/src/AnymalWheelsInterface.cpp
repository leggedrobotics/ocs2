/*
 * AnymalWheelsInterface.cpp
 *
 *  Created on: Nov 27, 2019
 *      Author: Marko Bjelonic
 */

#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h>
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsKinematics.h>

namespace anymal {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AnymalWheelsInterface::AnymalWheelsInterface(const std::string& pathToConfigFolder)
    : BASE(AnymalWheelsKinematics(), AnymalWheelsCom(), pathToConfigFolder) {
  // set up optimizers
  setupOptimizer(logicRulesPtr_, &defaultModeSequenceTemplate_, slqPtr_, mpcPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsInterface::setupOptimizer(const logic_rules_ptr_t& logicRulesPtr, const mode_sequence_template_t* modeSequenceTemplatePtr,
                                         slq_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) {
  dynamicsPtr_.reset(new system_dynamics_t(AnymalWheelsKinematicsAd(), AnymalWheelsComAd(), modelSettings_.recompileLibraries_));
  dynamicsDerivativesPtr_.reset(dynamicsPtr_->clone());
  constraintsPtr_.reset(new constraint_t(AnymalWheelsKinematicsAd(), AnymalWheelsComAd(), logicRulesPtr, modelSettings_));
  costFunctionPtr_.reset(new cost_function_t(AnymalWheelsCom(), logicRulesPtr, Q_, R_, QFinal_));
  operatingPointsPtr_.reset(new operating_point_t(AnymalWheelsCom(), logicRulesPtr));
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings_));

  // SLQ
  slqPtr.reset(new slq_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                         operatingPointsPtr_.get(), slqSettings_, logicRulesPtr));

  // MPC
  if (!modelSettings_.gaitOptimization_) {
    mpcPtr.reset(new mpc_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                           operatingPointsPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_, logicRulesPtr,
                           modeSequenceTemplatePtr));

  } else {
    throw std::runtime_error("mpc_ocs2 not configured");
  }
}

}  // end of namespace anymal
