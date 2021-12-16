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

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsEliminatePattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return s_filter.getAdiag().diagonal().cwiseProduct(x_filter) + s_filter.getBdiag().diagonal().cwiseProduct(u_filter);
  } else {
    vector_t dynamics_filter = s_filter.getA() * x_filter;
    dynamics_filter.noalias() += s_filter.getB() * u_filter;
    return dynamics_filter;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsEliminatePattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                           const PreComputation& preComp) {
  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system, preCompLS.getSystemPreComputation());

  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system.f, filterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(stateDim, stateDim);
  dynamics.dfdx.topLeftCorner(sysStateDim, sysStateDim) = dynamics_system.dfdx;
  dynamics.dfdx.bottomLeftCorner(filtStateDim, sysStateDim).setZero();
  if (isDiagonal) {
    dynamics.dfdx.topRightCorner(sysStateDim, filtStateDim).noalias() = dynamics_system.dfdu * s_filter.getCdiag();
  } else {
    dynamics.dfdx.topRightCorner(sysStateDim, filtStateDim).noalias() = dynamics_system.dfdu * s_filter.getC();
  }
  dynamics.dfdx.bottomRightCorner(filtStateDim, filtStateDim) = s_filter.getA();

  dynamics.dfdu.resize(stateDim, inputDim);
  if (isDiagonal) {
    dynamics.dfdu.topRows(sysStateDim).noalias() = dynamics_system.dfdu * s_filter.getDdiag();
  } else {
    dynamics.dfdu.topRows(sysStateDim).noalias() = dynamics_system.dfdu * s_filter.getD();
  }
  dynamics.dfdu.bottomRows(filtStateDim) = s_filter.getB();

  return dynamics;
}

}  // namespace ocs2
