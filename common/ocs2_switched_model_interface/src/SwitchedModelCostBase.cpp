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
    : BASE(Q, R, QFinal), comModelPtr_(comModel.clone()), modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)) {
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
scalar_t SwitchedModelCostBase::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  vector_t uNominal;
  const vector_t xNominal = costDesiredTrajectoriesPtr_->getDesiredState(t);
  inputFromContactFlags(contactFlags, xNominal, uNominal);

  vector_t xDeviation = x - xNominal;
  vector_t uDeviation = u - uNominal;
  return 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation) + uDeviation.dot(P_ * xDeviation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SwitchedModelCostBase::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  vector_t uNominal;
  const vector_t xNominal = costDesiredTrajectoriesPtr_->getDesiredState(t);
  inputFromContactFlags(contactFlags, xNominal, uNominal);

  vector_t xDeviation = x - xNominal;
  vector_t uDeviation = u - uNominal;

  ScalarFunctionQuadraticApproximation L;
  L.f = 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation) + uDeviation.dot(P_ * xDeviation);
  L.dfdx = Q_ * xDeviation + P_.transpose() * uDeviation;
  L.dfdu = R_ * uDeviation + P_ * xDeviation;
  L.dfdxx = Q_;
  L.dfdux = P_;
  L.dfduu = R_;
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::inputFromContactFlags(const contact_flag_t& contactFlags, const state_vector_t& nominalState,
                                                  vector_t& inputs) {
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
