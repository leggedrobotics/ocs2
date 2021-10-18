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

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsInputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsInputPattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return s_filter.getAdiag().diagonal().cwiseProduct(x_filter) + s_filter.getBdiag().diagonal().cwiseProduct(u_filter);
  } else {
    vector_t filterStateDerivative;
    filterStateDerivative.noalias() = s_filter.getA() * x_filter;
    filterStateDerivative.noalias() += s_filter.getB() * u_filter;
    return filterStateDerivative;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsInputPattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                       const PreComputation& preComp) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system, preCompLS.getSystemPreComputation());

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system.f, filterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(x.rows(), x.rows());
  dynamics.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = dynamics_system.dfdx;
  dynamics.dfdx.topRightCorner(x_system.rows(), x_filter.rows()).setZero();
  dynamics.dfdx.bottomLeftCorner(x_filter.rows(), x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(x_filter.rows(), x_filter.rows()) = s_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  dynamics.dfdu.topLeftCorner(x_system.rows(), u_system.rows()) = dynamics_system.dfdu;
  dynamics.dfdu.topRightCorner(x_system.rows(), u_filter.rows()).setZero();
  dynamics.dfdu.bottomLeftCorner(x_filter.rows(), u_system.rows()).setZero();
  dynamics.dfdu.bottomRightCorner(x_filter.rows(), u_filter.rows()) = s_filter.getB();

  return dynamics;
}

}  // namespace ocs2
