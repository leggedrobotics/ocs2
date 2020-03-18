/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_switched_model_interface/cost/SwitchedModelCostBase.h"

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelCostBase::SwitchedModelCostBase(const com_model_t& comModel, std::shared_ptr<const mode_schedule_manager_t> modeScheduleManagerPtr,
                                             const state_matrix_t& Q, const input_matrix_t& R, const state_matrix_t& QFinal)
    : BASE(Q, R, state_vector_t::Zero(), input_vector_t::Zero(), QFinal, state_vector_t::Zero()),
      comModelPtr_(comModel.clone()),
      modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)) {
  if (!modeScheduleManagerPtr_) {
    throw std::runtime_error("[SwitchedModelCostBase] logicRules cannot be a nullptr");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelCostBase::SwitchedModelCostBase(const SwitchedModelCostBase& rhs)
    : BASE(rhs), comModelPtr_(rhs.comModelPtr_->clone()), modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

SwitchedModelCostBase* SwitchedModelCostBase::clone() const {
  return new SwitchedModelCostBase(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
  // Get stance configuration
  contact_flag_t contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  dynamic_vector_t xNominal = state_vector_t::Zero();
  if (BASE::costDesiredTrajectoriesPtr_ != nullptr) {
    BASE::costDesiredTrajectoriesPtr_->getDesiredState(t, xNominal);
  }
  dynamic_vector_t uNominal;
  inputFromContactFlags(contactFlags, uNominal);

  BASE::setCurrentStateAndControl(t, x, u, xNominal, uNominal, xNominal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::inputFromContactFlags(contact_flag_t contactFlags, dynamic_vector_t& inputs) {
  // Distribute total mass equally over active stance legs.
  inputs.setZero(INPUT_DIM);

  const scalar_t totalMass = comModelPtr_->totalMass() * 9.81;
  size_t numStanceLegs(0);

  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    if (contactFlags[i]) {
      ++numStanceLegs;
    }
  }

  if (numStanceLegs > 0) {
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      if (contactFlags[i]) {
        inputs(3 * i + 2) = totalMass / numStanceLegs;
      }
    }
  }
}

}  // namespace switched_model
