//
// Created by rgrandia on 18.02.21.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace ocs2 {

enum class SensitivityIntegratorType { EULER, RK2, RK4 };

/**
 * A function handle that computes the discretized dynamics.
 * Returns x_{k+1}
 */
using DynamicsDiscretizer = std::function<vector_t(SystemDynamicsBase&, scalar_t, const vector_t&, const vector_t&, scalar_t)>;

/**
 * Select available integrator based on enum
 */
DynamicsDiscretizer selectDynamicsDiscretization(SensitivityIntegratorType integratorType);

/**
 * A function handle that computes the linear approximation of the discretized dynamics.
 * Returns an approximation of the form:
 *      x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 */
using DynamicsSensitivityDiscretizer =
    std::function<VectorFunctionLinearApproximation(SystemDynamicsBase&, scalar_t, const vector_t&, const vector_t&, scalar_t)>;

/**
 * Select available integrator based on enum
 */
DynamicsSensitivityDiscretizer selectDynamicsSensitivityDiscretization(SensitivityIntegratorType integratorType);

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
 * Computes the discretized dynamics. Uses an Runge-Kutta 2nd order discretization.
 * Returns x_{k+1}
 */
vector_t rk2Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt);

/**
 * Creates a linear approximation of the discretized dynamics. Uses an Runge-Kutta 2nd order discretization.
 * Returns an approximation of the form:
 *      x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 */
VectorFunctionLinearApproximation rk2SensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                               scalar_t dt);

/**
 * Computes the discretized dynamics. Uses an Runge-Kutta 4th order discretization.
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