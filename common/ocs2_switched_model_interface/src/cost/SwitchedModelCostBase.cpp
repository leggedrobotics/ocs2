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
    : ocs2::CostFunctionBase(rhs),
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
scalar_t SwitchedModelCostBase::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  update(t, x, u);

  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  const vector_t xNominal = costDesiredTrajectoriesPtr_->getDesiredState(t);
  vector_t uNominal = costDesiredTrajectoriesPtr_->getDesiredInput(t);
  inputFromContactFlags(contactFlags, xNominal, uNominal);
  // TODO (mspieler) : Same issue as in next function.
  if (uNominal.isZero()) {
    inputFromContactFlags(contactFlags, xNominal, uNominal);
  }

  vector_t xDeviation = x - xNominal;
  vector_t uDeviation = u - uNominal;

  return 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation) + footPlacementCost_->getCostValue();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwitchedModelCostBase::finalCost(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  const vector_t xDeviation = x - costDesiredTrajectoriesPtr_->getDesiredState(t);
  return 0.5 * xDeviation.dot(QFinal_ * xDeviation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SwitchedModelCostBase::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  update(t, x, u);

  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(t);

  const vector_t xNominal = costDesiredTrajectoriesPtr_->getDesiredState(t);
  vector_t uNominal = costDesiredTrajectoriesPtr_->getDesiredInput(t);
  // If the input has non-zero values, don't overwrite it.
  // TODO (rgrandia) : implement a better way to switch between heuristic inputs and tracking user defined inputs.
  // TODO (mspieler) : uNominal is always updated by costDesiredTrajectories.
  if (uNominal.isZero()) {
    inputFromContactFlags(contactFlags, xNominal, uNominal);
  }

  const vector_t xDeviation = x - xNominal;
  const vector_t uDeviation = u - uNominal;
  const vector_t qDeviation = Q_ * xDeviation;
  const vector_t rDeviation = R_ * uDeviation;

  ScalarFunctionQuadraticApproximation L;
  L.f = 0.5 * xDeviation.dot(qDeviation) + 0.5 * uDeviation.dot(rDeviation) + footPlacementCost_->getCostValue();
  L.dfdx = qDeviation + footPlacementCost_->getCostDerivativeState();
  L.dfdu = rDeviation;
  L.dfdxx = Q_ + footPlacementCost_->getCostSecondDerivativeState();
  L.dfdux.setZero(INPUT_DIM, STATE_DIM);
  L.dfduu = R_;
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::update(scalar_t t, const vector_t& x, const vector_t& u) {
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
ScalarFunctionQuadraticApproximation SwitchedModelCostBase::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  const vector_t xDeviation = x - costDesiredTrajectoriesPtr_->getDesiredState(t);
  const vector_t qDeviation = QFinal_ * xDeviation;

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = 0.5 * xDeviation.dot(qDeviation);
  Phi.dfdx = qDeviation;
  Phi.dfdxx = QFinal_;
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::inputFromContactFlags(const contact_flag_t& contactFlags, const vector_t& nominalState, vector_t& inputs) {
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
    const matrix3_t b_R_o = rotationMatrixOriginToBase(getOrientation(getComPose(switched_model::comkino_state_t(nominalState))));
    const vector3_t forceInBase = b_R_o * vector3_t{0.0, 0.0, totalMass / numStanceLegs};

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      if (contactFlags[i]) {
        inputs.segment<3>(3 * i) = forceInBase;
      }
    }
  }
}

}  // namespace switched_model
