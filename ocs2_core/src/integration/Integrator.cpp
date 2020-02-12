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
#include <unordered_map>

#include <ocs2_core/integration/Integrator.h>

namespace ocs2 {
namespace integrator_type {

std::string toString(IntegratorType integratorType) {
  static const std::unordered_map<IntegratorType, std::string> integratorMap = {
      {IntegratorType::EULER, "EULER"},
      {IntegratorType::ODE45, "ODE45"},
      {IntegratorType::ADAMS_BASHFORTH, "ADAMS_BASHFORTH"},
      {IntegratorType::BULIRSCH_STOER, "BULIRSCH_STOER"},
      {IntegratorType::MODIFIED_MIDPOINT, "MODIFIED_MIDPOINT"},
      {IntegratorType::RK4, "RK4"},
      {IntegratorType::RK5_VARIABLE, "RK5_VARIABLE"},
      {IntegratorType::ADAMS_BASHFORTH_MOULTON, "ADAMS_BASHFORTH_MOULTON"}};

  return integratorMap.at(integratorType);
}

IntegratorType fromString(const std::string& name) {
  static const std::unordered_map<std::string, IntegratorType> integratorMap = {
      {"EULER", IntegratorType::EULER},
      {"ODE45", IntegratorType::ODE45},
      {"ADAMS_BASHFORTH", IntegratorType::ADAMS_BASHFORTH},
      {"BULIRSCH_STOER", IntegratorType::BULIRSCH_STOER},
      {"MODIFIED_MIDPOINT", IntegratorType::MODIFIED_MIDPOINT},
      {"RK4", IntegratorType::RK4},
      {"RK5_VARIABLE", IntegratorType::RK5_VARIABLE},
      {"ADAMS_BASHFORTH_MOULTON", IntegratorType::ADAMS_BASHFORTH_MOULTON}};

  return integratorMap.at(name);
}

}  // namespace integrator_type
}  // namespace ocs2
