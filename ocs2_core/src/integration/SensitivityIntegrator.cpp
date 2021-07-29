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

#include "ocs2_core/integration/SensitivityIntegrator.h"

#include <unordered_map>

#include <ocs2_core/integration/SensitivityIntegratorImpl.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DynamicsDiscretizer selectDynamicsDiscretization(SensitivityIntegratorType integratorType) {
  switch (integratorType) {
    case SensitivityIntegratorType::EULER:
      return eulerDiscretization;
    case SensitivityIntegratorType::RK2:
      return rk2Discretization;
    case SensitivityIntegratorType::RK4:
      return rk4Discretization;
    default:
      throw std::runtime_error("Integrator of type " + sensitivity_integrator::toString(integratorType) + " not supported.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DynamicsSensitivityDiscretizer selectDynamicsSensitivityDiscretization(SensitivityIntegratorType integratorType) {
  switch (integratorType) {
    case SensitivityIntegratorType::EULER:
      return eulerSensitivityDiscretization;
    case SensitivityIntegratorType::RK2:
      return rk2SensitivityDiscretization;
    case SensitivityIntegratorType::RK4:
      return rk4SensitivityDiscretization;
    default:
      throw std::runtime_error("Integrator of type " + sensitivity_integrator::toString(integratorType) + " not supported.");
  }
}

namespace sensitivity_integrator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string toString(SensitivityIntegratorType integratorType) {
  static const std::unordered_map<SensitivityIntegratorType, std::string> integratorMap = {
      {SensitivityIntegratorType::EULER, "EULER"}, {SensitivityIntegratorType::RK2, "RK2"}, {SensitivityIntegratorType::RK4, "RK4"}};

  return integratorMap.at(integratorType);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SensitivityIntegratorType fromString(const std::string& name) {
  static const std::unordered_map<std::string, SensitivityIntegratorType> integratorMap = {
      {"EULER", SensitivityIntegratorType::EULER}, {"RK2", SensitivityIntegratorType::RK2}, {"RK4", SensitivityIntegratorType::RK4}};

  return integratorMap.at(name);
}

}  // namespace sensitivity_integrator

}  // namespace ocs2