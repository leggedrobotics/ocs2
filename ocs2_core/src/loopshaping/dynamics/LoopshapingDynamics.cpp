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

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsInputPattern.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamics::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input) {
  const vector_t systemstate = loopshapingDefinition_->getSystemState(state);
  const vector_t systeminput = loopshapingDefinition_->getSystemInput(state, input);
  const vector_t filterstate = loopshapingDefinition_->getFilterState(state);
  const vector_t filteredinput = loopshapingDefinition_->getFilteredInput(state, input);

  const vector_t systemstateDerivative = controlledSystem_->computeFlowMap(time, systemstate, systeminput);
  const vector_t filterstateDerivative = filterFlowmap(filterstate, filteredinput, systeminput);

  return loopshapingDefinition_->concatenateSystemAndFilterState(systemstateDerivative, filterstateDerivative);
}

vector_t LoopshapingDynamics::computeJumpMap(scalar_t time, const vector_t& state) {
  const vector_t systemstate = loopshapingDefinition_->getSystemState(state);
  const vector_t systemMappedState = controlledSystem_->computeJumpMap(time, systemstate);

  // Filter doesn't Jump
  const vector_t filterMappedState = loopshapingDefinition_->getFilterState(state);

  return loopshapingDefinition_->concatenateSystemAndFilterState(systemMappedState, filterMappedState);
}

vector_t LoopshapingDynamics::computeGuardSurfaces(scalar_t time, const vector_t& state) {
  const vector_t systemstate = loopshapingDefinition_->getSystemState(state);

  return controlledSystem_->computeGuardSurfaces(time, systemstate);
}

std::unique_ptr<LoopshapingDynamics> LoopshapingDynamics::create(const ControlledSystemBase& controlledSystem,
                                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingDynamics>(new LoopshapingDynamicsOutputPattern(controlledSystem, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingDynamics>(new LoopshapingDynamicsInputPattern(controlledSystem, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingDynamics>(
          new LoopshapingDynamicsEliminatePattern(controlledSystem, std::move(loopshapingDefinition)));
    default:
      throw std::runtime_error("[LoopshapingDynamics::create] invalid loopshaping type");
  }
}

}  // namespace ocs2
