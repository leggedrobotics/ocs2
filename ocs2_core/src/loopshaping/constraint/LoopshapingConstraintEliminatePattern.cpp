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
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintEliminatePattern.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LoopshapingConstraintEliminatePattern::getLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                const vector_t& u,
                                                                                                const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionLinearApproximation::Zero(0, x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = s_filter.getNumStates();

  // Not const so we can move
  auto g_system = StateInputConstraintCollection::getLinearApproximation(t, x_system, u_system, preComp_system);
  const auto numConstraints = g_system.f.rows();

  VectorFunctionLinearApproximation g;
  g.f = std::move(g_system.f);

  // dfdx
  g.dfdx.resize(numConstraints, stateDim);
  g.dfdx.leftCols(sysStateDim) = g_system.dfdx;
  if (isDiagonal) {
    g.dfdx.rightCols(filtStateDim).noalias() = g_system.dfdu * s_filter.getCdiag();
  } else {
    g.dfdx.rightCols(filtStateDim).noalias() = g_system.dfdu * s_filter.getC();
  }

  // dfdu
  if (isDiagonal) {
    g.dfdu.noalias() = g_system.dfdu * s_filter.getDdiag();
  } else {
    g.dfdu.noalias() = g_system.dfdu * s_filter.getD();
  }

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation LoopshapingConstraintEliminatePattern::getQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                      const vector_t& u,
                                                                                                      const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionQuadraticApproximation::Zero(0, x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
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
  if (isDiagonal) {
    h.dfdx.rightCols(filtStateDim).noalias() = h_system.dfdu * s_filter.getCdiag();
    h.dfdu.noalias() = h_system.dfdu * s_filter.getDdiag();
  } else {
    h.dfdx.rightCols(filtStateDim).noalias() = h_system.dfdu * s_filter.getC();
    h.dfdu.noalias() = h_system.dfdu * s_filter.getD();
  }

  h.dfdxx.resize(numConstraints);
  h.dfduu.resize(numConstraints);
  h.dfdux.resize(numConstraints);

  if (isDiagonal) {
    for (size_t i = 0; i < numConstraints; i++) {
      // dfdxx
      h.dfdxx[i].resize(stateDim, stateDim);
      h.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = h_system.dfdxx[i];
      h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim).noalias() = s_filter.getCdiag() * h_system.dfdux[i];
      h.dfdxx[i].topRightCorner(sysStateDim, filtStateDim) = h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim).transpose();
      h.dfdxx[i].bottomRightCorner(filtStateDim, filtStateDim) = s_filter.getScalingCdiagCdiag().cwiseProduct(h_system.dfduu[i]);

      // dfduu
      h.dfduu[i] = s_filter.getScalingDdiagDdiag().cwiseProduct(h_system.dfduu[i]);

      // dfdux
      h.dfdux[i].resize(inputDim, stateDim);
      h.dfdux[i].leftCols(sysStateDim).noalias() = s_filter.getDdiag() * h_system.dfdux[i];
      h.dfdux[i].rightCols(filtStateDim) = s_filter.getScalingDdiagCdiag().cwiseProduct(h_system.dfduu[i]);
    }

    return h;
  } else {
    matrix_t dfduu_C;  // temporary variable
    for (size_t i = 0; i < numConstraints; i++) {
      // dfdxx
      h.dfdxx[i].resize(stateDim, stateDim);
      h.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = h_system.dfdxx[i];
      h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim) = s_filter.getC().transpose() * h_system.dfdux[i];
      h.dfdxx[i].topRightCorner(sysStateDim, filtStateDim).noalias() = h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim).transpose();
      dfduu_C.noalias() = h_system.dfduu[i] * s_filter.getC();
      h.dfdxx[i].bottomRightCorner(filtStateDim, filtStateDim).noalias() = s_filter.getC().transpose() * dfduu_C;

      // dfduu
      h.dfduu[i].noalias() = s_filter.getD().transpose() * h_system.dfduu[i] * s_filter.getD();

      // dfdux
      h.dfdux[i].resize(inputDim, stateDim);
      h.dfdux[i].leftCols(sysStateDim).noalias() = s_filter.getD().transpose() * h_system.dfdux[i];
      h.dfdux[i].rightCols(filtStateDim).noalias() = s_filter.getD().transpose() * dfduu_C;
    }

    return h;
  }
}

}  // namespace ocs2
