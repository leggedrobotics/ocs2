//
// Created by rgrandia on 18.02.21.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace ocs2 {

/**
 * Computes the discretized dynamics. Uses an Forward euler discretization.
 * Returns x_{k+1}
 */
vector_t eulerDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt);

/**
 * Creates a linear approximation of the discretized dynamics. Uses an Forward euler discretization.
 * Returns an approximation of the form:
 *      x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 */
VectorFunctionLinearApproximation eulerSensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x,
                                                                 const vector_t& u, scalar_t dt);

/**
 * Computes the discretized dynamics. Uses an Forward euler discretization.
 * Returns x_{k+1}
 */
vector_t rk4Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt);

/**
 * Creates a linear approximation of the discretized dynamics. Uses an Runge-Kutta 4th order discretization.
 * Returns an approximation of the form:
 *      x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 */
VectorFunctionLinearApproximation rk4SensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                               scalar_t dt);

}  // namespace ocs2