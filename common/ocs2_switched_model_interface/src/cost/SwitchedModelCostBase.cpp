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
                                             const SwitchedModelModeScheduleManager& modeScheduleManager,
                                             const SwingTrajectoryPlanner& swingTrajectoryPlanner, const state_matrix_t& Q,
                                             const input_matrix_t& R, ModelSettings options)
    : comModelPtr_(comModel.clone()),
      footPlacementCost_(new FootPlacementCost(FootPlacementCostParameters(options.mu_, options.delta_),
                                               FootPlacementCostParameters(options.muSdf_, options.deltaSdf_), adComModel,
                                               adKinematicsModel, options.recompileLibraries_)),
      modeScheduleManagerPtr_(&modeScheduleManager),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      Q_(Q),
      R_(R) {}

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
      R_(rhs.R_) {}

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
  // If the input has non-zero values, don't overwrite it.
  // TODO (rgrandia) : implement a better way to switch between heuristic inputs and tracking user defined inputs.
  if (uNominal.isZero()) {
    uNominal = weightCompensatingInputs(*comModelPtr_, contactFlags, getOrientation(getComPose<scalar_t>(xNominal)));
  }

  const vector_t xDeviation = x - xNominal;
  const vector_t uDeviation = u - uNominal;
  return 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation) + footPlacementCost_->getCostValue();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwitchedModelCostBase::finalCost(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  return 0.0;
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
  if (uNominal.isZero()) {
    uNominal = weightCompensatingInputs(*comModelPtr_, contactFlags, getOrientation(getComPose<scalar_t>(xNominal)));
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
  L.dfdux.setZero(u.rows(), x.rows());
  L.dfduu = R_;
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCostBase::update(scalar_t t, const vector_t& x, const vector_t& u) {
  // Foot placement costs
  feet_array_t<const FootTangentialConstraintMatrix*> constraints = {{nullptr}};
  feet_array_t<SignedDistanceConstraint> sdfConstraints = {{nullptr, 0.0}};
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto& footPhase = swingTrajectoryPlannerPtr_->getFootPhase(leg, t);
    constraints[leg] = footPhase.getFootTangentialConstraintInWorldFrame();
    sdfConstraints[leg] = footPhase.getSignedDistanceConstraint(t);
  }
  footPlacementCost_->setStateAndConstraint(x, constraints, sdfConstraints);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SwitchedModelCostBase::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = 0.0;
  Phi.dfdx.setZero(STATE_DIM);
  Phi.dfdxx.setZero(STATE_DIM, STATE_DIM);
  return Phi;
}

}  // namespace switched_model
