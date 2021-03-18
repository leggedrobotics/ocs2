/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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