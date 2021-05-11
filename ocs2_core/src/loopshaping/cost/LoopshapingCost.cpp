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

#include <ocs2_core/loopshaping/cost/LoopshapingCost.h>
#include <ocs2_core/loopshaping/cost/LoopshapingStateCost.h>
#include <ocs2_core/loopshaping/cost/LoopshapingStateInputCost.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingCost::LoopshapingCost(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                                 std::shared_ptr<LoopshapingPreComputation> preCompPtr)
    : CostFunctionBase(systemCost, std::static_pointer_cast<PreComputation>(std::move(preCompPtr))) {
  // wrap the cost functions
  Base::costPtr_ = LoopshapingStateInputCost::create(*costPtr_, loopshapingDefinition);
  Base::finalCostPtr_.reset(new LoopshapingStateCost(*finalCostPtr_, loopshapingDefinition));
  Base::preJumpCostPtr_.reset(new LoopshapingStateCost(*preJumpCostPtr_, loopshapingDefinition));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingCost::LoopshapingCost(const LoopshapingCost& other, std::shared_ptr<PreComputation> preCompPtr)
    : CostFunctionBase(other, std::move(preCompPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBase* LoopshapingCost::clone(std::shared_ptr<PreComputation> preCompPtr) const {
  return new LoopshapingCost(*this, std::move(preCompPtr));
}

}  // namespace ocs2
