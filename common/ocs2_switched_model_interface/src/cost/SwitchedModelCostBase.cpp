/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_switched_model_interface/cost/SwitchedModelCostBase.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelCostBase::SwitchedModelCostBase(const MotionTrackingCost::Weights& trackingWeights,
                                             const SwitchedModelModeScheduleManager& modeScheduleManager,
                                             const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                             const com_model_t& comModel, bool recompile)
    : ocs2::CostFunctionBase(),
      trackingCostPtr_(new MotionTrackingCost(trackingWeights, modeScheduleManager, kinematicModel, adKinematicModel, comModel, recompile)),
      modeScheduleManagerPtr_(&modeScheduleManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelCostBase::SwitchedModelCostBase(const SwitchedModelCostBase& rhs)
    : ocs2::CostFunctionBase(rhs), trackingCostPtr_(rhs.trackingCostPtr_->clone()), modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

SwitchedModelCostBase* SwitchedModelCostBase::clone() const {
  return new SwitchedModelCostBase(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwitchedModelCostBase::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  if (targetTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] targetTrajectoriesPtr_ is not set");
  }

  return trackingCostPtr_->getValue(t, x, u, *targetTrajectoriesPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SwitchedModelCostBase::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (targetTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] targetTrajectoriesPtr_ is not set");
  }

  return trackingCostPtr_->getQuadraticApproximation(t, x, u, *targetTrajectoriesPtr_);
}

}  // namespace switched_model
