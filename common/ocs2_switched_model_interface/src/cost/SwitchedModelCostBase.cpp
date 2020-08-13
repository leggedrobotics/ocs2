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
SwitchedModelCostBase::SwitchedModelCostBase(const com_model_t& comModel, const ad_com_model_t& adComModel,
                                             const ad_kinematic_model_t& adKinematicsModel,
                                             std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                                             std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                                             const state_matrix_t& Q, const input_matrix_t& R, const state_matrix_t& QFinal,
                                             bool generateModels)
    : comModelPtr_(comModel.clone()),
      footPlacementCost_(new FootPlacementCost(FootPlacementCostParameters(), adComModel, adKinematicsModel, generateModels)),
      modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)),
      swingTrajectoryPlannerPtr_(std::move(swingTrajectoryPlannerPtr)),
      Q_(Q),
      R_(R),
      QFinal_(QFinal) {
  if (!modeScheduleManagerPtr_ || !swingTrajectoryPlannerPtr_) {
    throw std::runtime_error("[SwitchedModelCostBase] ModeScheduleManager and SwingTrajectoryPlanner cannot be a nullptr");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelCostBase::SwitchedModelCostBase(const SwitchedModelCostBase& rhs)
    : BASE(rhs),
      comModelPtr_(rhs.comModelPtr_->clone()),
      footPlacementCost_(rhs.footPlacementCost_->clone()),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      Q_(rhs.Q_),
      R_(rhs.R_),
      QFinal_(rhs.QFinal_) {}

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
  BASE::setCurrentStateAndControl(t, x, u);

  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  // References for quadratic term
  dynamic_vector_t xNominal = state_vector_t::Zero();
  dynamic_vector_t uNominal;
  if (BASE::costDesiredTrajectoriesPtr_ != nullptr) {
    BASE::costDesiredTrajectoriesPtr_->getDesiredState(t, xNominal);
    BASE::costDesiredTrajectoriesPtr_->getDesiredInput(t, uNominal);
  }
  // If the input has non-zero values, don't overwrite it.
  // TODO (rgrandia) : implement a better way to switch between heuristic inputs and tracking user defined inputs.
  if (uNominal.isZero()) {
    inputFromContactFlags(contactFlags, xNominal, uNominal);
  }

  inputFromContactFlags(contactFlags, xNominal, uNominal);
  xIntermediateDeviation_ = x - xNominal;
  uIntermediateDeviation_ = u - uNominal;
  xNominalFinal_ = xNominal;

  // Foot placement costs
  feet_array_t<const FootTangentialConstraintMatrix*> constraints = {{nullptr}};
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto& footPhase = swingTrajectoryPlannerPtr_->getFootPhase(leg, t);
    constraints[leg] = footPhase.getFootTangentialConstraintInWorldFrame();
  }
  footPlacementCost_->setStateAndConstraint(x, constraints);
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

void SwitchedModelCostBase::getIntermediateCost(scalar_t& L) {
  L = 0.5 * xIntermediateDeviation_.dot(Q_ * xIntermediateDeviation_) + 0.5 * uIntermediateDeviation_.dot(R_ * uIntermediateDeviation_);
  L += footPlacementCost_->getCostValue();
}

void SwitchedModelCostBase::getIntermediateCostDerivativeState(state_vector_t& dLdx) {
  dLdx = Q_ * xIntermediateDeviation_;
  dLdx += footPlacementCost_->getCostDerivativeState();
}

void SwitchedModelCostBase::getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) {
  dLdxx = Q_;
  dLdxx += footPlacementCost_->getCostSecondDerivativeState();
}

void SwitchedModelCostBase::getIntermediateCostDerivativeInput(input_vector_t& dLdu) {
  dLdu = R_ * uIntermediateDeviation_;
}

void SwitchedModelCostBase::getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) {
  dLduu = R_;
}

void SwitchedModelCostBase::getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) {
  dLdux.setZero();
}

void SwitchedModelCostBase::getTerminalCost(scalar_t& cost) {
  state_vector_t xFinalDeviation = BASE::x_ - xNominalFinal_;
  cost = 0.5 * xFinalDeviation.dot(QFinal_ * xFinalDeviation);
}

void SwitchedModelCostBase::getTerminalCostDerivativeState(state_vector_t& dPhidx) {
  state_vector_t xFinalDeviation = BASE::x_ - xNominalFinal_;
  dPhidx = QFinal_ * xFinalDeviation;
}

void SwitchedModelCostBase::getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) {
  dPhidxx = QFinal_;
}

}  // namespace switched_model
