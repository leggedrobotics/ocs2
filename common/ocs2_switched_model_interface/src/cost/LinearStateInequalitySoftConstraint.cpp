//
// Created by rgrandia on 26.07.21.
//

#include "ocs2_switched_model_interface/cost/LinearStateInequalitySoftconstraint.h"

namespace switched_model {

scalar_t getValue(const std::vector<LinearStateInequalitySoftConstraint>& constraints, const vector_t& f) {
  scalar_t cost(0.0);
  for (const auto& constraint : constraints) {
    for (int j = 0; j < constraint.h.size(); ++j) {  // Loop through all faces of the constraint
      cost += constraint.penalty->getValue(constraint.h(j));
    }
  }

  return cost;
}

ScalarFunctionQuadraticApproximation getQuadraticApproximation(const std::vector<LinearStateInequalitySoftConstraint>& constraints,
                                                               const vector_t& f, const matrix_t& dfdx) {
  const auto stateDim = dfdx.cols();
  const auto taskDim = dfdx.rows();

  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(stateDim);
  cost.dfdxx = matrix_t::Zero(stateDim, stateDim);

  vector_t taskSpaceDerivative = vector_t::Zero(taskDim);
  matrix_t taskSpaceSecondDerivative = matrix_t::Zero(taskDim, taskDim);

  for (const auto& constraint : constraints) {
    const auto& h = constraint.h;

    for (int j = 0; j < h.size(); ++j) {  // Loop through all faces of the constraint
      cost.f += constraint.penalty->getValue(h(j));
    }

    const auto penaltyDerivatives = h.unaryExpr([&](scalar_t hi) { return constraint.penalty->getDerivative(hi); });
    taskSpaceDerivative.noalias() += constraint.A.transpose() * penaltyDerivatives;

    const auto penaltySecondDerivatives = h.unaryExpr([&](scalar_t hi) { return constraint.penalty->getSecondDerivative(hi); });
    const matrix_t scaledConstraint = penaltySecondDerivatives.asDiagonal() * constraint.A;
    taskSpaceSecondDerivative.noalias() += constraint.A.transpose() * scaledConstraint;
  }

  cost.dfdx.noalias() = dfdx.transpose() * taskSpaceDerivative;

  const matrix_t scaledJacobian = taskSpaceSecondDerivative * dfdx;
  cost.dfdxx.noalias() = dfdx.transpose() * scaledJacobian;

  return cost;
}

}  // namespace switched_model
