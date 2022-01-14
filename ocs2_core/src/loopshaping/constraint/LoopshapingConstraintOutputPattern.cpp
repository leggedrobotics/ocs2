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

#include <ocs2_core/loopshaping/LoopshapingPreComputation.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintOutputPattern.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LoopshapingConstraintOutputPattern::getLinearApproximation(scalar_t t, const vector_t& x,
                                                                                             const vector_t& u,
                                                                                             const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionLinearApproximation::Zero(0, x.rows(), u.rows());
  }

  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  // Not const so we can move
  auto g_system = StateInputConstraintCollection::getLinearApproximation(t, x_system, u_system, preComp_system);
  const auto numConstraints = g_system.f.rows();

  VectorFunctionLinearApproximation g;
  g.f = std::move(g_system.f);

  g.dfdx.resize(numConstraints, stateDim);
  g.dfdx.leftCols(sysStateDim) = g_system.dfdx;
  g.dfdx.rightCols(filtStateDim).setZero();

  g.dfdu = std::move(g_system.dfdu);

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation LoopshapingConstraintOutputPattern::getQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                   const vector_t& u,
                                                                                                   const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionQuadraticApproximation::Zero(0, x.rows(), u.rows());
  }

  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  // Not const so we can move
  auto h_system = StateInputConstraintCollection::getQuadraticApproximation(t, x_system, u_system, preComp_system);
  const auto numConstraints = h_system.f.rows();

  VectorFunctionQuadraticApproximation h;
  h.f = std::move(h_system.f);

  h.dfdx.resize(numConstraints, stateDim);
  h.dfdx.leftCols(sysStateDim) = h_system.dfdx;
  h.dfdx.rightCols(filtStateDim).setZero();

  h.dfdu = std::move(h_system.dfdu);

  h.dfdxx.resize(numConstraints);
  h.dfduu.resize(numConstraints);
  h.dfdux.resize(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    h.dfdxx[i].setZero(stateDim, stateDim);
    h.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = h_system.dfdxx[i];

    h.dfduu[i] = std::move(h_system.dfduu[i]);

    h.dfdux[i].resize(inputDim, stateDim);
    h.dfdux[i].leftCols(sysStateDim) = h_system.dfdux[i];
    h.dfdux[i].rightCols(filtStateDim).setZero();
  }

  return h;
}

}  // namespace ocs2
