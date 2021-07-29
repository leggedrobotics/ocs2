/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <memory>
#include <string>

#include <ocs2_core/integration/IntegratorBase.h>

namespace ocs2 {

/**
 * @brief The IntegratorType enum
 * Enum used in selecting a specific integrator.
 */
enum class IntegratorType {
  EULER,
  ODE45,
  ODE45_OCS2,
  ADAMS_BASHFORTH,
  BULIRSCH_STOER,
  MODIFIED_MIDPOINT,
  RK4,
  RK5_VARIABLE,
  ADAMS_BASHFORTH_MOULTON
};

namespace integrator_type {

/**
 * Get string name of integrator type
 * @param [in] integratorType: Integrator type enum
 */
std::string toString(IntegratorType integratorType);

/**
 * Get integrator type from string name, useful for reading config file
 * @param [in] name: Integrator name
 */
IntegratorType fromString(const std::string& name);

}  // namespace integrator_type

/**
 * Create Integrator of given type.
 *
 * @param [in] integratorType: The integrator type.
 * @param [in] eventHandler: The integration event function.
 */
std::unique_ptr<IntegratorBase> newIntegrator(IntegratorType integratorType,
                                              const std::shared_ptr<SystemEventHandler>& eventHandlerPtr = nullptr);

}  // namespace ocs2
