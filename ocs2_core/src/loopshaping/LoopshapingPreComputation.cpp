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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingPreComputation::LoopshapingPreComputation(const PreComputation& systemPreComputation,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
    : systemPreCompPtr_(systemPreComputation.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingPreComputation::LoopshapingPreComputation(const LoopshapingPreComputation& other)
    : loopshapingDefinition_(other.loopshapingDefinition_) {
  systemPreCompPtr_.reset(other.systemPreCompPtr_->clone());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingPreComputation* LoopshapingPreComputation::clone() const {
  return new LoopshapingPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  loopshapingDefinition_->getSystemState(x, systemState_);
  loopshapingDefinition_->getSystemInput(x, u, systemInput_);
  loopshapingDefinition_->getFilterState(x, filterState_);
  loopshapingDefinition_->getFilteredInput(x, u, filterInput_);

  systemPreCompPtr_->request(request, t, systemState_, systemInput_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingPreComputation::requestPreJump(RequestSet request, scalar_t t, const vector_t& x) {
  loopshapingDefinition_->getSystemState(x, systemState_);
  loopshapingDefinition_->getFilterState(x, filterState_);

  systemPreCompPtr_->requestPreJump(request, t, systemState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingPreComputation::requestFinal(RequestSet request, scalar_t t, const vector_t& x) {
  loopshapingDefinition_->getSystemState(x, systemState_);
  loopshapingDefinition_->getFilterState(x, filterState_);

  systemPreCompPtr_->requestFinal(request, t, systemState_);
}

}  // namespace ocs2
