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

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/LoopshapingFilter.h>

namespace ocs2 {

enum class LoopshapingType { outputpattern, inputpattern, eliminatepattern };

/*
 *  Class to assemble and store the loopshaping definition
 *  The definition contains two filters
 *  r: filters on inputs
 *  s: filters on inputs (implemented as inverse)
 */
class LoopshapingDefinition {
 public:
  LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, scalar_t gamma = 0.9)
      : gamma_(gamma), loopshapingType_(loopshapingType), filter_(std::move(filter)) {
    diagonal_ = true;
    diagonal_ = diagonal_ && filter_.getA().isDiagonal();
    diagonal_ = diagonal_ && filter_.getB().isDiagonal();
    diagonal_ = diagonal_ && filter_.getC().isDiagonal();
    diagonal_ = diagonal_ && filter_.getD().isDiagonal();
  }

  LoopshapingType getType() const { return loopshapingType_; };
  bool isDiagonal() const { return diagonal_; };
  const Filter& getInputFilter() const { return filter_; };

  void print() const { filter_.print(); };

  vector_t getSystemState(const vector_t& state) const { return state.head(state.rows() - filter_.getNumStates()); };

  vector_t getSystemInput(const vector_t& state, const vector_t& input) const {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern: /* fall through */
      case LoopshapingType::inputpattern:
        if (input.rows() > filter_.getNumInputs()) {
          return input.head(input.rows() - filter_.getNumInputs());
        } else {
          return input;
        }
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

  vector_t getFilterState(const vector_t& state) const { return state.tail(filter_.getNumStates()); };

  vector_t getFilteredInput(const vector_t& state, const vector_t& input) const {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern:
        if (diagonal_) {
          return filter_.getCdiag().diagonal().cwiseProduct(state.tail(filter_.getNumStates())) +
                 filter_.getDdiag().diagonal().cwiseProduct(input.head(filter_.getNumInputs()));
        } else {
          vector_t u = filter_.getC() * state.tail(filter_.getNumStates());
          u.noalias() += filter_.getD() * input.head(filter_.getNumInputs());
          return u;
        }
      case LoopshapingType::inputpattern: /* fall through */
      case LoopshapingType::eliminatepattern:
        return input.tail(filter_.getNumInputs());
      default:
        throw std::runtime_error("[LoopshapingDefinition::getFilteredInput] invalid loopshaping type");
    }
  };

  vector_t concatenateSystemAndFilterState(const vector_t& systemState, const vector_t& filterState) const {
    vector_t state(systemState.rows() + filter_.getNumStates());
    state << systemState, filterState;
    return state;
  };

  vector_t concatenateSystemAndFilterInput(const vector_t& systemInput, const vector_t& filterInput) const {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern:
        return systemInput;
      case LoopshapingType::inputpattern: {
        vector_t input(systemInput.rows() + filterInput.rows());
        input << systemInput, filterInput;
        return input;
      }
      case LoopshapingType::eliminatepattern:
        return filterInput;
      default:
        throw std::runtime_error("[LoopshapingDefinition::concatenateSystemAndFilterInput] invalid loopshaping type");
    }
  };

  void getFilterEquilibrium(const vector_t& systemInput, vector_t& filterState, vector_t& filterInput) const {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern:
        // When systemInput is the input to the filter
        filter_.findEquilibriumForInput(systemInput, filterState, filterInput);
        break;
      case LoopshapingType::inputpattern: /* fall through */
      case LoopshapingType::eliminatepattern:
        // When systemInput is the output of the filter
        filter_.findEquilibriumForOutput(systemInput, filterState, filterInput);
        break;
      default:
        throw std::runtime_error("[LoopshapingDefinition::getFilterEquilibrium] invalid loopshaping type");
    }
  }

 public:
  scalar_t gamma_;

 private:
  Filter filter_;
  LoopshapingType loopshapingType_;
  bool diagonal_;
};

}  // namespace ocs2
