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
 * Implementation based on the QR decomposition
 *
 * @param constraint : C = dfdx, D = dfdu, e = f;
 * @return Px = dfdx, Pu = dfdu, Pe = f;
 */
VectorFunctionLinearApproximation qrConstraintProjection(const VectorFunctionLinearApproximation& constraint);

/**
 * Returns the linear projection
 *  u = Pu * \tilde{u} + Px * x + Pe
 *
 * s.t. C*x + D*u + e = 0 is satisfied for any \tilde{u}
 *
 * Implementation based on the LU decomposition
 *
 * @param constraint : C = dfdx, D = dfdu, e = f;
 * @return Px = dfdx, Pu = dfdu, Pe = f;
 */
VectorFunctionLinearApproximation luConstraintProjection(const VectorFunctionLinearApproximation& constraint);

}  // namespace ocs2