//
// Created by rgrandia on 03.03.21.
//

#include "ocs2_sqp/ConstraintProjection.h"

namespace ocs2 {

VectorFunctionLinearApproximation qrConstraintProjection(const VectorFunctionLinearApproximation& constraint) {
  // Constraint Projectors are based on the QR decomposition
  const auto numConstraints = constraint.dfdu.rows();
  const auto numInputs = constraint.dfdu.cols();
  Eigen::HouseholderQR<ocs2::matrix_t> QRof_DT(constraint.dfdu.transpose());

  auto RT = QRof_DT.matrixQR().topRows(numConstraints).triangularView<Eigen::Upper>().transpose();
  ocs2::matrix_t RTinvC = RT.solve(constraint.dfdx);  // inv(R^T) * C
  ocs2::matrix_t RTinve = RT.solve(constraint.f);     // inv(R^T) * e

  ocs2::matrix_t Q = QRof_DT.householderQ();
  auto Q1 = Q.leftCols(numConstraints);

  VectorFunctionLinearApproximation projectionTerms;
  projectionTerms.dfdu = Q.rightCols(numInputs - numConstraints);
  projectionTerms.dfdx.noalias() = -Q1 * RTinvC;
  projectionTerms.f.noalias() = -Q1 * RTinve;

  return projectionTerms;
}

VectorFunctionLinearApproximation luConstraintProjection(const VectorFunctionLinearApproximation& constraint) {
  // Constraint Projectors are based on the LU decomposition
  Eigen::FullPivLU<ocs2::matrix_t> lu(constraint.dfdu);

  VectorFunctionLinearApproximation projectionTerms;
  projectionTerms.dfdu = lu.kernel();
  projectionTerms.dfdx.noalias() = -lu.solve(constraint.dfdx);
  projectionTerms.f.noalias() = -lu.solve(constraint.f);

  return projectionTerms;
}

}  // namespace ocs2