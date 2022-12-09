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
#include <ocs2_core/integration/RungeKuttaDormandPrince5.h>
#include <ocs2_core/integration/implementation/Integrator.h>

namespace ocs2 {

namespace integrator_type {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string toString(IntegratorType integratorType) {
  static const std::unordered_map<IntegratorType, std::string> integratorMap = {
      {IntegratorType::EULER, "EULER"},
      {IntegratorType::ODE45, "ODE45"},
      {IntegratorType::ODE45_OCS2, "ODE45_OCS2"},
      {IntegratorType::ADAMS_BASHFORTH, "ADAMS_BASHFORTH"},
      {IntegratorType::BULIRSCH_STOER, "BULIRSCH_STOER"},
      {IntegratorType::MODIFIED_MIDPOINT, "MODIFIED_MIDPOINT"},
      {IntegratorType::RK4, "RK4"},
      {IntegratorType::RK5_VARIABLE, "RK5_VARIABLE"},
      {IntegratorType::ADAMS_BASHFORTH_MOULTON, "ADAMS_BASHFORTH_MOULTON"}};

  return integratorMap.at(integratorType);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
IntegratorType fromString(const std::string& name) {
  static const std::unordered_map<std::string, IntegratorType> integratorMap = {
      {"EULER", IntegratorType::EULER},
      {"ODE45", IntegratorType::ODE45},
      {"ODE45_OCS2", IntegratorType::ODE45_OCS2},
      {"ADAMS_BASHFORTH", IntegratorType::ADAMS_BASHFORTH},
      {"BULIRSCH_STOER", IntegratorType::BULIRSCH_STOER},
      {"MODIFIED_MIDPOINT", IntegratorType::MODIFIED_MIDPOINT},
      {"RK4", IntegratorType::RK4},
      {"RK5_VARIABLE", IntegratorType::RK5_VARIABLE},
      {"ADAMS_BASHFORTH_MOULTON", IntegratorType::ADAMS_BASHFORTH_MOULTON}};

  return integratorMap.at(name);
}

}  // namespace integrator_type

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<IntegratorBase> newIntegrator(IntegratorType integratorType, const std::shared_ptr<SystemEventHandler>& eventHandlerPtr) {
  switch (integratorType) {
    case (IntegratorType::EULER):
      return std::make_unique<IntegratorEuler>(eventHandlerPtr);
    case (IntegratorType::ODE45):
      return std::make_unique<ODE45>(eventHandlerPtr);
    case (IntegratorType::ODE45_OCS2):
      return std::make_unique<RungeKuttaDormandPrince5>(eventHandlerPtr);
    case (IntegratorType::ADAMS_BASHFORTH):
      return std::make_unique<IntegratorAdamsBashforth<1>>(eventHandlerPtr);
    case (IntegratorType::BULIRSCH_STOER):
      return std::make_unique<IntegratorBulirschStoer>(eventHandlerPtr);
    case (IntegratorType::MODIFIED_MIDPOINT):
      return std::make_unique<IntegratorModifiedMidpoint>(eventHandlerPtr);
    case (IntegratorType::RK4):
      return std::make_unique<IntegratorRK4>(eventHandlerPtr);
    case (IntegratorType::RK5_VARIABLE):
      return std::make_unique<IntegratorRK5Variable>(eventHandlerPtr);
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
    case (IntegratorType::ADAMS_BASHFORTH_MOULTON):
      return std::make_unique<IntegratorAdamsBashforthMoulton<1>>(eventHandlerPtr);
#endif
    default:
      throw std::runtime_error("Integrator of type " + integrator_type::toString(integratorType) + " not supported.");
  }
}

}  // namespace ocs2
