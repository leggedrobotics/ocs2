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

#include "ocs2_core/loopshaping/initialization/LoopshapingInitializer.h"

#include <iostream>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingInitializer::LoopshapingInitializer(const Initializer& systembase, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
    : systembase_(systembase.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingInitializer::LoopshapingInitializer(const LoopshapingInitializer& other)
    : Initializer(other), systembase_(other.systembase_->clone()), loopshapingDefinition_(other.loopshapingDefinition_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingInitializer* LoopshapingInitializer::clone() const {
  return new LoopshapingInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  // system state-input initializer
  vector_t systemInput, systemNextState;
  const vector_t systemState = loopshapingDefinition_->getSystemState(state);
  systembase_->compute(time, systemState, nextTime, systemInput, systemNextState);

  // Compute input
  vector_t equilibriumFilterInput;
  const vector_t filterState = loopshapingDefinition_->getFilterState(state);
  loopshapingDefinition_->getFilterEquilibriumGivenState(systemInput, filterState, equilibriumFilterInput);
  input = loopshapingDefinition_->augmentedSystemInput(systemInput, equilibriumFilterInput);

  // Next filter state
  vector_t filterNextState = filterState + (nextTime - time) * loopshapingDefinition_->filterFlowMap(filterState, input);
  nextState = loopshapingDefinition_->concatenateSystemAndFilterState(systemNextState, filterNextState);
}

}  // namespace ocs2
