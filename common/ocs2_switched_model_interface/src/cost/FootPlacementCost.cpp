//
// Created by rgrandia on 26.06.20.
//

#include "ocs2_switched_model_interface/cost/FootPlacementCost.h"

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/cost/LinearStateInequalitySoftconstraint.h"

namespace switched_model {

FootPlacementCost::FootPlacementCost(ocs2::RelaxedBarrierPenalty::Config settings)
    : polygonPenalty_(new ocs2::RelaxedBarrierPenalty(settings)) {}

FootPlacementCost::FootPlacementCost(const FootPlacementCost& rhs) : polygonPenalty_(rhs.polygonPenalty_->clone()) {}

FootPlacementCost* FootPlacementCost::clone() const {
  return new FootPlacementCost(*this);
}

scalar_t FootPlacementCost::getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                                     const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);

  scalar_t cost(0.0);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = switchedModelPreComp.getFootTangentialConstraintInWorldFrame(leg);

    if (constraintMatrixPtr != nullptr) {
      const auto& footPosition = switchedModelPreComp.footPositionInOriginFrame(leg);

      LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
      linearStateInequalitySoftConstraint.penalty = polygonPenalty_.get();
      linearStateInequalitySoftConstraint.A = constraintMatrixPtr->A;
      linearStateInequalitySoftConstraint.h = constraintMatrixPtr->b;
      linearStateInequalitySoftConstraint.h.noalias() += constraintMatrixPtr->A * footPosition;

      cost += switched_model::getValue(linearStateInequalitySoftConstraint, footPosition);
    }
  }

  return cost;
}

ScalarFunctionQuadraticApproximation FootPlacementCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                  const ocs2::TargetTrajectories& targetTrajectories,
                                                                                  const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);

  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = switchedModelPreComp.getFootTangentialConstraintInWorldFrame(leg);

    if (constraintMatrixPtr != nullptr) {
      const auto& footPosition = switchedModelPreComp.footPositionInOriginFrame(leg);
      const auto& footJacobian = switchedModelPreComp.footPositionInOriginFrameStateDerivative(leg);

      LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
      linearStateInequalitySoftConstraint.penalty = polygonPenalty_.get();
      linearStateInequalitySoftConstraint.A = constraintMatrixPtr->A;
      linearStateInequalitySoftConstraint.h = constraintMatrixPtr->b;
      linearStateInequalitySoftConstraint.h.noalias() += constraintMatrixPtr->A * footPosition;

      const auto targetcost = switched_model::getQuadraticApproximation(linearStateInequalitySoftConstraint, footPosition, footJacobian);
      cost.f += targetcost.f;
      cost.dfdx += targetcost.dfdx;
      cost.dfdxx += targetcost.dfdxx;
    }
  }

  return cost;
}

}  // namespace switched_model
