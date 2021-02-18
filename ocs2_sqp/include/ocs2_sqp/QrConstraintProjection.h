//
// Created by rgrandia on 18.02.21.
//

#pragma once

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Returns the linear projection
 *  u = Pu * \tilde{u} + Px * x + Pe
 *
 * s.t. C*x + D*u + e = 0 is satisfied for any \tilde{u}
 *
 * @param constraint : C = dfdx, D = dfdu, e = f;
 * @return Px = dfdx, Pu = dfdu, Pe = f;
 */
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

}  // namespace ocs2