//
// Created by rgrandia on 26.07.21.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_core/penalties/penalties/PenaltyBase.h>

namespace switched_model {
/**
 * Linear inequality constraint in task space: A * f(x) + b >= 0
 * h(x) = A * f(x) + b
 * together with the penalty function to apply to this constraint
 */
struct LinearStateInequalitySoftConstraint {
  Eigen::Matrix<scalar_t, -1, 3> A;
  vector_t h;
  const ocs2::PenaltyBase* penalty;
};

/// Specialization where there is only 1 constraint
struct SingleLinearStateInequalitySoftConstraint {
  Eigen::Matrix<scalar_t, 1, 3> A;
  scalar_t h;
  const ocs2::PenaltyBase* penalty;
};

scalar_t getValue(const LinearStateInequalitySoftConstraint& constraints, const vector_t& f);

scalar_t getValue(const SingleLinearStateInequalitySoftConstraint& constraints, const vector_t& f);

ScalarFunctionQuadraticApproximation getQuadraticApproximation(const LinearStateInequalitySoftConstraint& constraints, const vector_t& f,
                                                               const matrix_t& dfdx);

ScalarFunctionQuadraticApproximation getQuadraticApproximation(const SingleLinearStateInequalitySoftConstraint& constraint,
                                                               const vector_t& f, const matrix_t& dfdx);

}  // namespace switched_model
