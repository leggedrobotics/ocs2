//
// Created by rgrandia on 26.06.20.
//

#include "ocs2_switched_model_interface/cost/CollisionAvoidanceCost.h"

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/cost/LinearStateInequalitySoftconstraint.h"

namespace switched_model {

CollisionAvoidanceCost::CollisionAvoidanceCost(ocs2::RelaxedBarrierPenalty::Config settings)
    : penalty_(new ocs2::RelaxedBarrierPenalty(settings)) {}

CollisionAvoidanceCost::CollisionAvoidanceCost(const CollisionAvoidanceCost& rhs) : penalty_(rhs.penalty_->clone()) {}

CollisionAvoidanceCost* CollisionAvoidanceCost::clone() const {
  return new CollisionAvoidanceCost(*this);
}

scalar_t CollisionAvoidanceCost::getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                                          const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto* sdfPtr = switchedModelPreComp.getSignedDistanceField();

  scalar_t cost(0.0);
  if (sdfPtr != nullptr) {
    const auto& collisionSpheresActive = switchedModelPreComp.collisionSpheresActive();
    const auto& collisionSpheres = switchedModelPreComp.collisionSpheresInOriginFrame();

    for (size_t i = 0; i < collisionSpheres.size(); ++i) {
      if (collisionSpheresActive[i]) {
        const auto& collisionSphere = collisionSpheres[i];
        const auto sdfDistance = sdfPtr->value(collisionSphere.position);
        const auto h_sdf = sdfDistance - collisionSphere.radius;

        SingleLinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
        linearStateInequalitySoftConstraint.penalty = penalty_.get();
        // linearStateInequalitySoftConstraint.A = Leave empty
        linearStateInequalitySoftConstraint.h = h_sdf;

        cost += switched_model::getValue(linearStateInequalitySoftConstraint, collisionSphere.position);
      }
    }
  }

  return cost;
}

ScalarFunctionQuadraticApproximation CollisionAvoidanceCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const ocs2::TargetTrajectories& targetTrajectories,
                                                                                       const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);

  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);

  const auto* sdfPtr = switchedModelPreComp.getSignedDistanceField();

  if (sdfPtr != nullptr) {
    const auto& collisionSpheresActive = switchedModelPreComp.collisionSpheresActive();
    const auto& collisionSpheres = switchedModelPreComp.collisionSpheresInOriginFrame();
    const auto& collisionSphereDerivatives = switchedModelPreComp.collisionSpheresInOriginFrameStateDerivative();
    for (size_t i = 0; i < collisionSpheres.size(); ++i) {
      if (collisionSpheresActive[i]) {
        const auto& collisionSphere = collisionSpheres[i];
        const auto& collisionSphereDerivative = collisionSphereDerivatives[i];
        const auto sdfFirstOrder = sdfPtr->valueAndDerivative(collisionSphere.position);
        const auto h_sdf = sdfFirstOrder.first - collisionSphere.radius;

        SingleLinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
        linearStateInequalitySoftConstraint.penalty = penalty_.get();
        linearStateInequalitySoftConstraint.A = sdfFirstOrder.second.transpose();
        linearStateInequalitySoftConstraint.h = h_sdf;

        const auto targetcost = switched_model::getQuadraticApproximation(linearStateInequalitySoftConstraint, collisionSphere.position,
                                                                          collisionSphereDerivative);
        cost.f += targetcost.f;
        cost.dfdx += targetcost.dfdx;
        cost.dfdxx += targetcost.dfdxx;
      }
    }
  }

  return cost;
}

}  // namespace switched_model
