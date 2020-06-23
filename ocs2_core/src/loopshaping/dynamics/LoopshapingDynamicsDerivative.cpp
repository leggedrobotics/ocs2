/******************************************************************************
Copyright (c) 2020, Ruben Grandia. All rights reserved.

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

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivative.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivativeEliminatePattern.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivativeInputPattern.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivativeOutputPattern.h>

namespace ocs2 {

void LoopshapingDynamicsDerivative::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  systemApproximationValid_ = false;
  jumpMapApproximationValid_ = false;
  BASE::setCurrentStateAndControl(t, x, u);

  const vector_t systemstate = loopshapingDefinition_->getSystemState(x);
  const vector_t systeminput = loopshapingDefinition_->getSystemInput(x, u);
  systemDerivative_->setCurrentStateAndControl(t, systemstate, systeminput);
}

vector_t LoopshapingDynamicsDerivative::getFlowMapDerivativeTime() {
  const vector_t system_df = systemDerivative_->getFlowMapDerivativeTime();
  const vector_t filter_df = vector_t::Zero(loopshapingDefinition_->getInputFilter().getNumStates());
  return loopshapingDefinition_->concatenateSystemAndFilterState(system_df, filter_df);
}

matrix_t LoopshapingDynamicsDerivative::getFlowMapDerivativeState() {
  computeSystemDerivatives();
  return loopshapingFlowMapDerivativeState();
}

matrix_t LoopshapingDynamicsDerivative::getFlowMapDerivativeInput() {
  computeSystemDerivatives();
  return loopshapingFlowMapDerivativeInput();
}

vector_t LoopshapingDynamicsDerivative::getJumpMapDerivativeTime() {
  const vector_t system_dg = systemDerivative_->getJumpMapDerivativeTime();
  const vector_t filter_dg = vector_t::Zero(loopshapingDefinition_->getInputFilter().getNumStates());
  return loopshapingDefinition_->concatenateSystemAndFilterState(system_dg, filter_dg);
}

matrix_t LoopshapingDynamicsDerivative::getJumpMapDerivativeState() {
  computeJumpMapDerivatives();
  return loopshapingJumpMapDerivativeState();
}

matrix_t LoopshapingDynamicsDerivative::getJumpMapDerivativeInput() {
  computeJumpMapDerivatives();
  return loopshapingJumpMapDerivativeInput();
}

vector_t LoopshapingDynamicsDerivative::getGuardSurfacesDerivativeTime() {
  throw std::runtime_error("[LoopshapingDynamicsDerivative] Guard surfaces not implemented");
}

matrix_t LoopshapingDynamicsDerivative::getGuardSurfacesDerivativeState() {
  throw std::runtime_error("[LoopshapingDynamicsDerivative] Guard surfaces not implemented");
}

matrix_t LoopshapingDynamicsDerivative::getGuardSurfacesDerivativeInput() {
  throw std::runtime_error("[LoopshapingDynamicsDerivative] Guard surfaces not implemented");
}

void LoopshapingDynamicsDerivative::computeSystemDerivatives() {
  if (!systemApproximationValid_) {
    A_system_ = systemDerivative_->getFlowMapDerivativeState();
    B_system_ = systemDerivative_->getFlowMapDerivativeInput();
    systemApproximationValid_ = true;
  }
}

void LoopshapingDynamicsDerivative::computeJumpMapDerivatives() {
  if (!jumpMapApproximationValid_) {
    G_system_ = systemDerivative_->getJumpMapDerivativeState();
    H_system_ = systemDerivative_->getJumpMapDerivativeInput();
    jumpMapApproximationValid_ = true;
  }
}

std::unique_ptr<LoopshapingDynamicsDerivative> LoopshapingDynamicsDerivative::create(
    const DerivativesBase& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingDynamicsDerivative>(
          new LoopshapingDynamicsDerivativeOutputPattern(controlledSystem, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingDynamicsDerivative>(
          new LoopshapingDynamicsDerivativeInputPattern(controlledSystem, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingDynamicsDerivative>(
          new LoopshapingDynamicsDerivativeEliminatePattern(controlledSystem, std::move(loopshapingDefinition)));
    default:
      throw std::runtime_error("[LoopshapingDynamicsDerivative::create] invalid loopshaping type");
  }
}

}  // namespace ocs2
