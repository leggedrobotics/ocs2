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

#include "ocs2_core/augmented_lagrangian/StateInputAugmentedLagrangianCollection.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputAugmentedLagrangianCollection* StateInputAugmentedLagrangianCollection::clone() const {
  return new StateInputAugmentedLagrangianCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateInputAugmentedLagrangianCollection::getNumberOfActiveConstraints(scalar_t time) const {
  size_t numConstraints = 0;
  for (const auto& term : terms_) {
    if (term->isActive(time)) {
      numConstraints += term->getNumConstraints(time);
    }
  }
  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<LagrangianMetrics> StateInputAugmentedLagrangianCollection::getValue(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const std::vector<Multiplier>& termsMultiplier,
                                                                                 const PreComputation& preComp) const {
  std::vector<LagrangianMetrics> termsConstraintPenalty;
  termsConstraintPenalty.reserve(terms_.size());
  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      termsConstraintPenalty.emplace_back(terms_[i]->getValue(time, state, input, termsMultiplier[i], preComp));
    } else {
      termsConstraintPenalty.emplace_back(0.0, vector_t());
    }
  }
  return termsConstraintPenalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputAugmentedLagrangianCollection::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const std::vector<Multiplier>& termsMultiplier,
    const PreComputation& preComp) const {
  const auto firstActiveItr =
      std::find_if(terms_.begin(), terms_.end(),
                   [time](const std::unique_ptr<StateInputAugmentedLagrangianInterface>& costTerm) { return costTerm->isActive(time); });

  // no active terms (or terms is empty).
  if (firstActiveItr == terms_.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());
  }

  // initialize with first active term
  const size_t firstActiveInd = std::distance(terms_.begin(), firstActiveItr);
  auto penalty = (*firstActiveItr)->getQuadraticApproximation(time, state, input, termsMultiplier[firstActiveInd], preComp);

  // accumulate terms
  for (size_t i = firstActiveInd + 1; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      penalty += terms_[i]->getQuadraticApproximation(time, state, input, termsMultiplier[i], preComp);
    }
  }

  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputAugmentedLagrangianCollection::updateLagrangian(scalar_t time, const vector_t& state, const vector_t& input,
                                                               std::vector<LagrangianMetrics>& termsMetrics,
                                                               std::vector<Multiplier>& termsMultiplier) const {
  assert(termsMetrics.size() == termsMultiplier.size());

  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      Multiplier updatedLagrangian;
      std::tie(updatedLagrangian, termsMetrics[i].penalty) =
          terms_[i]->updateLagrangian(time, state, input, termsMetrics[i].constraint, termsMultiplier[i]);
      termsMultiplier[i] = std::move(updatedLagrangian);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputAugmentedLagrangianCollection::initializeLagrangian(scalar_t time, std::vector<Multiplier>& termsMultiplier) const {
  termsMultiplier.clear();
  termsMultiplier.reserve(terms_.size());
  for (const auto& term : terms_) {
    if (term->isActive(time)) {
      termsMultiplier.emplace_back(term->initializeLagrangian(time));
    } else {
      termsMultiplier.emplace_back(0.0, vector_t());
    }
  }
}

}  // namespace ocs2
