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

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/LoopshapingFilter.h>

namespace ocs2 {

/**
 * Loopshaping types:
 *      outputpattern : The system inputs remain the inputs of the augmented system.
 *                      Loopshaping inputs are a linear combination of state and system inputs
 *      eliminatepattern : The loopshaping inputs become the inputs of the augmented system.
 *                         The system inputs are retreived through a linear combination of state and system inputs.
 */
enum class LoopshapingType { outputpattern, eliminatepattern };

/**
 *  Class to store and access the loopshaping definition
 */
class LoopshapingDefinition {
 public:
  LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, matrix_t costMatrix = matrix_t());

  LoopshapingType getType() const { return loopshapingType_; }
  bool isDiagonal() const { return diagonal_; }
  const Filter& getInputFilter() const { return filter_; }

  scalar_t loopshapingCost(const vector_t& filteredInput) const { return 0.5 * filteredInput.dot(R_ * filteredInput); }

  matrix_t& costMatrix() { return R_; }

  void print() const;

  vector_t getSystemState(const vector_t& state) const { return state.head(state.rows() - filter_.getNumStates()); }

  vector_t getSystemInput(const vector_t& state, const vector_t& input) const;

  vector_t getFilterState(const vector_t& state) const { return state.tail(filter_.getNumStates()); }

  vector_t getFilteredInput(const vector_t& state, const vector_t& input) const;

  vector_t concatenateSystemAndFilterState(const vector_t& systemState, const vector_t& filterState) const;

  vector_t augmentedSystemInput(const vector_t& systemInput, const vector_t& filterInput) const;

  void getFilterEquilibrium(const vector_t& systemInput, vector_t& filterState, vector_t& filterInput) const;

 private:
  Filter filter_;
  LoopshapingType loopshapingType_;
  bool diagonal_;
  matrix_t R_;
};

}  // namespace ocs2
