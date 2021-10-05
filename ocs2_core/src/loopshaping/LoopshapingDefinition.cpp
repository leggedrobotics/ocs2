/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

#include <iostream>

namespace ocs2 {

LoopshapingDefinition::LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, matrix_t costMatrix)
    : loopshapingType_(loopshapingType), filter_(std::move(filter)), R_(std::move(costMatrix)) {
  // Detect diagonal formulation if all involved matrices are diagonal
  diagonal_ = filter_.getA().isDiagonal() && filter_.getB().isDiagonal() && filter_.getC().isDiagonal() && filter_.getD().isDiagonal();

  if (R_.size() == 0) {  // No cost provided
    R_.setIdentity(filter_.getNumInputs(), filter_.getNumInputs());
  }
}

void LoopshapingDefinition::print() const {
  std::cerr << "[LoopshapingDefinition] \n";
  filter_.print();
  std::cerr << "R on filtered inputs: \n" << R_ << std::endl;
};

vector_t LoopshapingDefinition::getSystemInput(const vector_t& state, const vector_t& input) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern:
      return input;
    case LoopshapingType::eliminatepattern: {
      if (diagonal_) {
        return filter_.getCdiag().diagonal().cwiseProduct(state.tail(filter_.getNumStates())) +
               filter_.getDdiag().diagonal().cwiseProduct(input);
      } else {
        // u = C*x + D*v. Use noalias to prevent temporaries.
        vector_t u = filter_.getC() * state.tail(filter_.getNumStates());
        u.noalias() += filter_.getD() * input;
        return u;
      }
    }
    default:
      throw std::runtime_error("[LoopshapingDefinition::getSystemInput] invalid loopshaping type");
  }
};

vector_t LoopshapingDefinition::getFilteredInput(const vector_t& state, const vector_t& input) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern:
      if (diagonal_) {
        return filter_.getCdiag().diagonal().cwiseProduct(state.tail(filter_.getNumStates())) +
               filter_.getDdiag().diagonal().cwiseProduct(input);
      } else {
        vector_t u = filter_.getC() * state.tail(filter_.getNumStates());
        u.noalias() += filter_.getD() * input;
        return u;
      }
    case LoopshapingType::eliminatepattern:
      return input;
    default:
      throw std::runtime_error("[LoopshapingDefinition::getFilteredInput] invalid loopshaping type");
  }
};

vector_t LoopshapingDefinition::concatenateSystemAndFilterState(const vector_t& systemState, const vector_t& filterState) const {
  vector_t state(systemState.rows() + filter_.getNumStates());
  state << systemState, filterState;
  return state;
}

vector_t LoopshapingDefinition::augmentedSystemInput(const vector_t& systemInput, const vector_t& filterInput) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern:
      return systemInput;
    case LoopshapingType::eliminatepattern:
      return filterInput;
    default:
      throw std::runtime_error("[LoopshapingDefinition::augmentedSystemInput] invalid loopshaping type");
  }
}

void LoopshapingDefinition::getFilterEquilibrium(const vector_t& systemInput, vector_t& filterState, vector_t& filterInput) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern:
      filter_.findEquilibriumForInput(systemInput, filterState, filterInput);
      break;
    case LoopshapingType::eliminatepattern:
      filter_.findEquilibriumForOutput(systemInput, filterState, filterInput);
      break;
    default:
      throw std::runtime_error("[LoopshapingDefinition::getFilterEquilibrium] invalid loopshaping type");
  }
}

}  // namespace ocs2