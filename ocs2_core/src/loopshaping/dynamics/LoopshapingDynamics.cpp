/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamics::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) {
  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();

  const vector_t dynamics_system = systemDynamics_->computeFlowMap(time, x_system, u_system, preComp_system);
  const vector_t dynamics_filter = filterFlowmap(x_filter, u_filter, u_system);

  return loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system, dynamics_filter);
}

vector_t LoopshapingDynamics::computeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp) {
  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();

  const vector_t jumpMap_system = systemDynamics_->computeJumpMap(time, x_system, preComp_system);

  // Filter doesn't Jump
  const auto& jumMap_filter = preCompLS.getFilterState();

  return loopshapingDefinition_->concatenateSystemAndFilterState(jumpMap_system, jumMap_filter);
}

vector_t LoopshapingDynamics::computeGuardSurfaces(scalar_t time, const vector_t& state) {
  const vector_t x_system = loopshapingDefinition_->getSystemState(state);
  return systemDynamics_->computeGuardSurfaces(time, x_system);
}

VectorFunctionLinearApproximation LoopshapingDynamics::jumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                  const PreComputation& preComp) {
  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);

  // System jump
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto jumpMap_system = systemDynamics_->jumpMapLinearApproximation(t, x_system, preComp_system);

  // Filter doesn't Jump
  const auto& jumMap_filter = preCompLS.getFilterState();

  VectorFunctionLinearApproximation jumpMap;
  jumpMap.f = loopshapingDefinition_->concatenateSystemAndFilterState(jumpMap_system.f, jumMap_filter);

  const auto postJumpStateDim = jumpMap.f.rows();
  const auto postJumpSysStateDim = jumpMap_system.f.rows();
  const auto postJumpFiltStateDim = jumMap_filter.rows();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = stateDim - sysStateDim;

  jumpMap.dfdx.resize(postJumpStateDim, stateDim);
  jumpMap.dfdx.topLeftCorner(postJumpSysStateDim, sysStateDim) = jumpMap_system.dfdx;
  jumpMap.dfdx.bottomLeftCorner(postJumpFiltStateDim, sysStateDim).setZero();
  jumpMap.dfdx.topRightCorner(postJumpSysStateDim, filtStateDim).setZero();
  jumpMap.dfdx.bottomRightCorner(postJumpFiltStateDim, filtStateDim).setIdentity();

  jumpMap.dfdu.resize(postJumpStateDim, 0);

  return jumpMap;
}

VectorFunctionLinearApproximation LoopshapingDynamics::guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  throw std::runtime_error("[LoopshapingDynamics] Guard surfaces not implemented");
}

vector_t LoopshapingDynamics::flowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  const vector_t system_df = systemDynamics_->flowMapDerivativeTime(t, x, u);
  const vector_t filter_df = vector_t::Zero(loopshapingDefinition_->getInputFilter().getNumStates());
  return loopshapingDefinition_->concatenateSystemAndFilterState(system_df, filter_df);
}

vector_t LoopshapingDynamics::jumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  const vector_t system_dg = systemDynamics_->jumpMapDerivativeTime(t, x, u);
  const vector_t filter_dg = vector_t::Zero(loopshapingDefinition_->getInputFilter().getNumStates());
  return loopshapingDefinition_->concatenateSystemAndFilterState(system_dg, filter_dg);
}

vector_t LoopshapingDynamics::guardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  throw std::runtime_error("[LoopshapingDynamics] Guard surfaces not implemented");
}

std::unique_ptr<LoopshapingDynamics> LoopshapingDynamics::create(const SystemDynamicsBase& systemDynamics,
                                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  // wrap the system pre-computation
  LoopshapingPreComputation preComputation(systemDynamics.getPreComputation(), loopshapingDefinition);

  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingDynamics>(
          new LoopshapingDynamicsOutputPattern(systemDynamics, std::move(loopshapingDefinition), preComputation));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingDynamics>(
          new LoopshapingDynamicsEliminatePattern(systemDynamics, std::move(loopshapingDefinition), preComputation));
    default:
      throw std::runtime_error("[LoopshapingDynamics::create] invalid loopshaping type");
  }
}

}  // namespace ocs2
