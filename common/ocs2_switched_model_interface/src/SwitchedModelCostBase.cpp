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
SwitchedModelCostBase::SwitchedModelCostBase(const com_model_t& comModel,
                                             std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                                             const state_matrix_t& Q, const input_matrix_t& R, const state_matrix_t& QFinal)
    : BASE(Q, R, vector_t::Zero(STATE_DIM), vector_t::Zero(INPUT_DIM), QFinal, vector_t::Zero(STATE_DIM)),
      comModelPtr_(comModel.clone()),
      modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)) {
  if (!modeScheduleManagerPtr_) {
    throw std::runtime_error("[SwitchedModelCostBase] Mode schedule manager cannot be a nullptr");
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
std::pair<vector_t, vector_t> SwitchedModelCostBase::getNominalStateInput(scalar_t t, const vector_t& x, const vector_t& u) {
  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  vector_t xNominal = vector_t::Zero(STATE_DIM);
  if (BASE::costDesiredTrajectoriesPtr_ != nullptr) {
    xNominal = BASE::costDesiredTrajectoriesPtr_->getDesiredState(t);
  }
  vector_t uNominal = inputFromContactFlags(contactFlags);

  return {xNominal, uNominal};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SwitchedModelCostBase::inputFromContactFlags(contact_flag_t contactFlags) {
  // Distribute total mass equally over active stance legs.
  vector_t inputs = vector_t::Zero(INPUT_DIM);

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
  return inputs;
}

}  // namespace switched_model
