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
#include <ocs2_core/loopshaping/constraint/LoopshapingStateConstraint.h>

#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintEliminatePattern.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintOutputPattern.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t LoopshapingStateConstraint::getValue(scalar_t t, const vector_t& x, const PreComputation& preComp) const {
  if (this->empty()) {
    return vector_array_t();
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();

  return StateConstraintCollection::getValue(t, x_system, preComp_system);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LoopshapingStateConstraint::getLinearApproximation(scalar_t t, const vector_t& x,
                                                                                     const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionLinearApproximation::Zero(0, x.rows());
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  // Not const so we can move
  auto c_system = StateConstraintCollection::getLinearApproximation(t, x_system, preComp_system);
  const auto numConstraints = c_system.f.rows();

  VectorFunctionLinearApproximation c;

  c.f = std::move(c_system.f);

  c.dfdx.resize(numConstraints, stateDim);
  c.dfdx.leftCols(sysStateDim) = c_system.dfdx;
  c.dfdx.rightCols(filtStateDim).setZero();

  return c;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation LoopshapingStateConstraint::getQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                           const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionQuadraticApproximation::Zero(0, x.rows());
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  // Not const so we can move
  const auto c_system = StateConstraintCollection::getQuadraticApproximation(t, x_system, preComp_system);
  const auto numConstraints = c_system.f.rows();

  VectorFunctionQuadraticApproximation c;

  c.f = std::move(c_system.f);

  c.dfdx.resize(numConstraints, stateDim);
  c.dfdx.leftCols(sysStateDim) = c_system.dfdx;
  c.dfdx.rightCols(filtStateDim).setZero();

  c.dfdxx.resize(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    c.dfdxx[i].setZero(stateDim, stateDim);
    c.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = c_system.dfdxx[i];
  }

  return c;
}

}  // namespace ocs2
