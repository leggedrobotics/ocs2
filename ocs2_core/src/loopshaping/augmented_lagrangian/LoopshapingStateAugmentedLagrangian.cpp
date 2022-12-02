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
#include <ocs2_core/loopshaping/augmented_lagrangian/LoopshapingStateAugmentedLagrangian.h>

namespace ocs2 {

std::vector<LagrangianMetrics> LoopshapingStateAugmentedLagrangian::getValue(scalar_t t, const vector_t& x,
                                                                             const std::vector<Multiplier>& termsMultiplier,
                                                                             const PreComputation& preComp) const {
  if (this->empty()) {
    return {};
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();

  return StateAugmentedLagrangianCollection::getValue(t, x_system, termsMultiplier, preComp_system);
}

ScalarFunctionQuadraticApproximation LoopshapingStateAugmentedLagrangian::getQuadraticApproximation(
    scalar_t t, const vector_t& x, const std::vector<Multiplier>& termsMultiplier, const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows());
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  auto Phi_system = StateAugmentedLagrangianCollection::getQuadraticApproximation(t, x_system, termsMultiplier, preComp_system);

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = std::move(Phi_system.f);
  Phi.dfdx.setZero(stateDim);
  Phi.dfdx.head(sysStateDim) = Phi_system.dfdx;
  Phi.dfdxx.setZero(stateDim, stateDim);
  Phi.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = Phi_system.dfdxx;
  return Phi;
}

void LoopshapingStateAugmentedLagrangian::updateLagrangian(scalar_t t, const vector_t& x, std::vector<LagrangianMetrics>& termsMetrics,
                                                           std::vector<Multiplier>& termsMultiplier) const {
  if (this->empty()) {
    termsMultiplier.clear();
  }

  const auto x_system = loopshapingDefinition_->getSystemState(x);
  StateAugmentedLagrangianCollection::updateLagrangian(t, x_system, termsMetrics, termsMultiplier);
}

}  // namespace ocs2
