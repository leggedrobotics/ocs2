/*
 * OCS2AnymalInterface.cpp
 *
 *  Created on: May 11, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OCS2AnymalInterface::OCS2AnymalInterface(const std::string& pathToConfigFolder)
    : BASE(AnymalKinematics(), AnymalCom(), pathToConfigFolder) {
  // set up optimizers
  setupOptimizer(logicRulesPtr_, &defaultModeSequenceTemplate_, slqPtr_, mpcPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::setupOptimizer(const logic_rules_ptr_t& logicRulesPtr, const mode_sequence_template_t* modeSequenceTemplatePtr,
                                         slq_base_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) {
  dynamicsPtr_ = std::unique_ptr<system_dynamics_t>(new system_dynamics_t(modelSettings_.recompileLibraries_));
  dynamicsDerivativesPtr_.reset(dynamicsPtr_->clone());
  constraintsPtr_ = std::unique_ptr<constraint_t>(new constraint_t(logicRulesPtr, modelSettings_));
  costFunctionPtr_ = std::unique_ptr<cost_function_t>(new cost_function_t(logicRulesPtr, Q_, R_, QFinal_));

  generalized_coordinate_t defaultCoordinate = initRbdState_.template head<18>();
  operatingPointsPtr_ = std::unique_ptr<operating_point_t>(new operating_point_t(logicRulesPtr, modelSettings_, defaultCoordinate));

  // SLQ
  if (slqSettings_.ddpSettings_.useMultiThreading_) {
    slqPtr = slq_base_ptr_t(new slq_mp_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                         operatingPointsPtr_.get(), slqSettings_, logicRulesPtr));
  } else {
    slqPtr = slq_base_ptr_t(new slq_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                      operatingPointsPtr_.get(), slqSettings_, logicRulesPtr));
  }

  // MPC
  if (!modelSettings_.gaitOptimization_) {
    mpcPtr = mpc_ptr_t(new mpc_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
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
