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
SwitchedModelCostBase::SwitchedModelCostBase(const com_model_t& comModel,
                                             std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                                             const state_matrix_t& Q, const input_matrix_t& R, const state_matrix_t& QFinal)
    : BASE(Q, R, state_vector_t::Zero(), input_vector_t::Zero(), QFinal, state_vector_t::Zero()),
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

/******************************************************************************************************
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  dynamic_vector_t xNominal = state_vector_t::Zero();
  dynamic_vector_t uNominal;
  if (BASE::costDesiredTrajectoriesPtr_ != nullptr) {
    BASE::costDesiredTrajectoriesPtr_->getDesiredState(t, xNominal);
    BASE::costDesiredTrajectoriesPtr_->getDesiredInput(t, uNominal);
  }
  // If the input has non-zero values, don't overwrite it.
  if (uNominal.isZero()) {
    inputFromContactFlags(contactFlags, xNominal, uNominal);
  };

  BASE::setCurrentStateAndControl(t, x, u, xNominal, uNominal, xNominal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::inputFromContactFlags(const contact_flag_t& contactFlags, const state_vector_t& nominalState,
                                                  dynamic_vector_t& inputs) {
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
    const matrix3_t b_R_o = rotationMatrixOriginToBase(getOrientation(getComPose(nominalState)));
    const vector3_t forceInBase = b_R_o * vector3_t{0.0, 0.0, totalMass / numStanceLegs};

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      if (contactFlags[i]) {
        inputs.segment<3>(3 * i) = forceInBase;
      }
    }
  }
}

}  // namespace switched_model
