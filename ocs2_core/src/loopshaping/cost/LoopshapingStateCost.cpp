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
#include <ocs2_core/loopshaping/cost/LoopshapingStateCost.h>

namespace ocs2 {

scalar_t LoopshapingStateCost::getValue(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories,
                                        const PreComputation& preComp) const {
  if (this->empty()) {
    return 0.0;
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();

  return StateCostCollection::getValue(t, x_system, targetTrajectories, preCompLS.getSystemPreComputation());
}

ScalarFunctionQuadraticApproximation LoopshapingStateCost::getQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                     const TargetTrajectories& targetTrajectories,
                                                                                     const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), 0);
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  const auto Phi_system =
      StateCostCollection::getQuadraticApproximation(t, x_system, targetTrajectories, preCompLS.getSystemPreComputation());

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = Phi_system.f;
  Phi.dfdx.resize(stateDim);
  Phi.dfdx.head(sysStateDim) = Phi_system.dfdx;
  Phi.dfdx.tail(filtStateDim).setZero();
  Phi.dfdxx.setZero(stateDim, stateDim);
  Phi.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = Phi_system.dfdxx;
  return Phi;
}

}  // namespace ocs2
