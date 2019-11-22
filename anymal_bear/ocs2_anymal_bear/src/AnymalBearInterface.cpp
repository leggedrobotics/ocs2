/*
 * AnymalBearInterface.cpp
 *
 *  Created on: May 11, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_bear/AnymalBearInterface.h"

#include <ocs2_anymal_bear_switched_model/core/AnymalCom.h>
#include <ocs2_anymal_bear_switched_model/core/AnymalKinematics.h>

namespace anymal {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AnymalBearInterface::AnymalBearInterface(const std::string& pathToConfigFolder)
    : BASE(AnymalKinematics(), AnymalCom(), pathToConfigFolder) {
  // set up optimizers
  setupOptimizer(logicRulesPtr_, &defaultModeSequenceTemplate_, slqPtr_, mpcPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalBearInterface::setupOptimizer(const logic_rules_ptr_t& logicRulesPtr, const mode_sequence_template_t* modeSequenceTemplatePtr,
                                         slq_base_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) {
  dynamicsPtr_.reset(new system_dynamics_t(AnymalKinematicsAd(), AnymalComAd(), modelSettings_.recompileLibraries_));
  dynamicsDerivativesPtr_.reset(dynamicsPtr_->clone());
  constraintsPtr_.reset(new constraint_t(AnymalKinematicsAd(), AnymalComAd(), logicRulesPtr, modelSettings_));
  costFunctionPtr_.reset(new cost_function_t(AnymalCom(), logicRulesPtr, Q_, R_, QFinal_));
  operatingPointsPtr_.reset(new operating_point_t(AnymalCom(), logicRulesPtr));
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings_));

  // SLQ
  if (slqSettings_.ddpSettings_.useMultiThreading_) {
    slqPtr.reset(new slq_mp_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                         operatingPointsPtr_.get(), slqSettings_, logicRulesPtr));
  } else {
    slqPtr.reset(new slq_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                      operatingPointsPtr_.get(), slqSettings_, logicRulesPtr));
  }

  // MPC
  if (!modelSettings_.gaitOptimization_) {
    mpcPtr.reset(new mpc_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                 operatingPointsPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_, logicRulesPtr,
                                 modeSequenceTemplatePtr));

  } else {
    throw std::runtime_error("mpc_ocs2 not configured");
    //		typedef ocs2::MPC_OCS2<BASE::state_dim_, BASE::input_dim_>	mpc_ocs2_t;
    //
    //		mpcPtr = mpc_ptr_t( new mpc_ocs2_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
    //				costFunctionPtr_.get(), operatingPointsPtr_.get(),
    //				partitioningTimes_,
    //				slqSettings_, mpcSettings_, logicRulesPtr, modeSequenceTemplatePtr));
  }
}

}  // end of namespace anymal
