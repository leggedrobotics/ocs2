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

enum class SensitivityIntegratorType { EULER, RK2, RK4 };

namespace sensitivity_integrator {

/**
 * Get string name of integrator type
 * @param integratorType: Integrator type enum
 */
std::string toString(SensitivityIntegratorType integratorType);

/**
 * Get integrator type from string name, useful for reading config file
 * @param name: Integrator name
 */
SensitivityIntegratorType fromString(const std::string& name);

}  // namespace sensitivity_integrator

/**
 * A function handle to computes the discrete approximation of the system's flowmap.
 * @param system : system to be discretized
 * @param t : starting time of the discretization interval
 * @param x : starting state x_{k}
 * @param u : input u_{k}, assumed constant over the entire interval
 * @param dt : interval duration
 * Returns x_{k+1}
 */
using DynamicsDiscretizer = std::function<vector_t(SystemDynamicsBase&, scalar_t, const vector_t&, const vector_t&, scalar_t)>;

/**
 * Select available integrator based on enum
 */
DynamicsDiscretizer selectDynamicsDiscretization(SensitivityIntegratorType integratorType);

/**
 * A function handle to compute the linear approximation of the discretized system's flowmap.
 *
 * @param system : system to be discretized
 * @param t : starting time of the discretization interval
 * @param x : starting state x_{k}
 * @param u : input u_{k}, assumed constant over the entire interval
 * @param dt : interval duration
 * Returns an approximation of the form:
 *      x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 */
using DynamicsSensitivityDiscretizer =
    std::function<VectorFunctionLinearApproximation(SystemDynamicsBase&, scalar_t, const vector_t&, const vector_t&, scalar_t)>;

/**
 * Select available integrator based on enum
 */
DynamicsSensitivityDiscretizer selectDynamicsSensitivityDiscretization(SensitivityIntegratorType integratorType);

}  // namespace ocs2