/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/cost/StateInputCostCollection.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostCollection::StateInputCostCollection(const StateInputCostCollection& other) : Collection<StateInputCost>(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostCollection* StateInputCostCollection::clone() const {
  return new StateInputCostCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputCostCollection::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                            const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  scalar_t cost = 0.0;

  // accumulate cost terms
  for (const auto& costTerm : this->terms_) {
    if (costTerm->isActive(time)) {
      cost += costTerm->getValue(time, state, input, targetTrajectories, preComp);
    }
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputCostCollection::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const vector_t& input,
                                                                                         const TargetTrajectories& targetTrajectories,
                                                                                         const PreComputation& preComp) const {
  const auto firstActive = std::find_if(terms_.begin(), terms_.end(),
                                        [time](const std::unique_ptr<StateInputCost>& costTerm) { return costTerm->isActive(time); });

  // No active terms (or terms is empty).
  if (firstActive == terms_.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.rows(), input.rows());
  }

  // Initialize with first active term, accumulate potentially other active terms.
  auto cost = (*firstActive)->getQuadraticApproximation(time, state, input, targetTrajectories, preComp);
  std::for_each(std::next(firstActive), terms_.end(), [&](const std::unique_ptr<StateInputCost>& costTerm) {
    if (costTerm->isActive(time)) {
      cost += costTerm->getQuadraticApproximation(time, state, input, targetTrajectories, preComp);
    }
  });

  return cost;
}

}  // namespace ocs2
