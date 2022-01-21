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

#include "ocs2_core/augmented_lagrangian/StateAugmentedLagrangianCollection.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangianCollection* StateAugmentedLagrangianCollection::clone() const {
  return new StateAugmentedLagrangianCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<vector_t, scalar_t>> StateAugmentedLagrangianCollection::getValue(scalar_t time, const vector_t& state,
                                                                                        const std::vector<Multiplier>& termsMultiplier,
                                                                                        const PreComputation& preComp) const {
  std::vector<std::pair<vector_t, scalar_t>> termsConstraintPenalty;
  termsConstraintPenalty.reserve(terms_.size());
  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      termsConstraintPenalty.emplace_back(terms_[i]->getValue(time, state, termsMultiplier[i], preComp));
    }
  }
  return termsConstraintPenalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateAugmentedLagrangianCollection::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const std::vector<Multiplier>& termsMultiplier, const PreComputation& preComp) const {
  const auto firstActiveItr =
      std::find_if(terms_.begin(), terms_.end(),
                   [time](const std::unique_ptr<StateAugmentedLagrangianInterface>& costTerm) { return costTerm->isActive(time); });

  // no active terms (or terms is empty).
  if (firstActiveItr == terms_.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.size(), 0);
  }

  // initialize with first active term
  const size_t firstActiveInd = std::distance(terms_.begin(), firstActiveItr);
  auto penalty = (*firstActiveItr)->getQuadraticApproximation(time, state, termsMultiplier[firstActiveInd], preComp);

  // accumulate terms
  for (size_t i = firstActiveInd + 1; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      penalty += terms_[i]->getQuadraticApproximation(time, state, termsMultiplier[i], preComp);
    }
  }

  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateAugmentedLagrangianCollection::updateLagrangian(const scalar_t& time, const vector_t& state,
                                                          std::vector<std::pair<vector_t, scalar_t>>& termsConstraintPenalty,
                                                          std::vector<Multiplier>& termsMultiplier) const {
  assert(termsConstraintPenalty.size() == termsMultiplier.size());

  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      Multiplier updatedLagrangian;
      std::tie(updatedLagrangian, termsConstraintPenalty[i].second) =
          terms_[i]->updateLagrangian(time, state, termsConstraintPenalty[i].first, termsMultiplier[i]);
      termsMultiplier[i] = std::move(updatedLagrangian);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateAugmentedLagrangianCollection::initializeLagrangian(scalar_t time, std::vector<Multiplier>& termsMultiplier) const {
  termsMultiplier.clear();
  termsMultiplier.reserve(terms_.size());
  for (const auto& term : terms_) {
    if (term->isActive(time)) {
      termsMultiplier.emplace_back(term->initializeLagrangian(time));
    }
  }
}

}  // namespace ocs2
