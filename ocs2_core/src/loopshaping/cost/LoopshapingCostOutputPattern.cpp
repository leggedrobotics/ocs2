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
#include <ocs2_core/loopshaping/cost/LoopshapingCostOutputPattern.h>

namespace ocs2 {

ScalarFunctionQuadraticApproximation LoopshapingCostOutputPattern::getQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                             const vector_t& u,
                                                                                             const TargetTrajectories& targetTrajectories,
                                                                                             const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();
  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  const auto& Rfilter = loopshapingDefinition_->costMatrix();
  const vector_t Ru_filter = Rfilter * u_filter;

  // Not const, so we can move
  auto L_system =
      StateInputCostCollection::getQuadraticApproximation(t, x_system, u_system, targetTrajectories, preCompLS.getSystemPreComputation());

  ScalarFunctionQuadraticApproximation L;
  L.f = L_system.f + 0.5 * u_filter.dot(Ru_filter);

  L.dfdx.resize(stateDim);
  L.dfdx.head(sysStateDim) = L_system.dfdx;
  if (isDiagonal) {
    L.dfdx.tail(filtStateDim).noalias() = r_filter.getCdiag().diagonal().cwiseProduct(Ru_filter);
  } else {
    L.dfdx.tail(filtStateDim).noalias() = r_filter.getC().transpose() * Ru_filter;
  }

  L.dfdxx.setZero(stateDim, stateDim);
  L.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = L_system.dfdxx;
  matrix_t dfduu_C;  // temporary variable, reused in other equation.
  if (isDiagonal) {
    dfduu_C.noalias() = Rfilter * r_filter.getCdiag();
    L.dfdxx.bottomRightCorner(filtStateDim, filtStateDim).noalias() = r_filter.getCdiag() * dfduu_C;
  } else {
    dfduu_C.noalias() = Rfilter * r_filter.getC();
    L.dfdxx.bottomRightCorner(filtStateDim, filtStateDim).noalias() = r_filter.getC().transpose() * dfduu_C;
  }

  L.dfdu = std::move(L_system.dfdu);
  L.dfduu = std::move(L_system.dfduu);
  if (isDiagonal) {
    L.dfdu.noalias() += r_filter.getDdiag().diagonal().cwiseProduct(Ru_filter);
    L.dfduu.noalias() += r_filter.getDdiag() * Rfilter * r_filter.getDdiag();
  } else {
    L.dfdu.noalias() += r_filter.getD().transpose() * Ru_filter;
    L.dfduu.noalias() += r_filter.getD().transpose() * Rfilter * r_filter.getD();
  }

  L.dfdux.resize(inputDim, stateDim);
  L.dfdux.leftCols(sysStateDim) = L_system.dfdux;
  if (isDiagonal) {
    L.dfdux.rightCols(filtStateDim).noalias() = r_filter.getDdiag() * dfduu_C;
  } else {
    L.dfdux.rightCols(filtStateDim).noalias() = r_filter.getD().transpose() * dfduu_C;
  }

  return L;
}

}  // namespace ocs2
