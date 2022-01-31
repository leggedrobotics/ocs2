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
  /**
   * Constructor
   * @param loopshapingType : Type of loopshaping system augmentation to apply
   * @param filter : Filter encoding the dynamics between system and loopshaping (=filtered) inputs
   * @param costMatrix : Quadratic cost term to apply to the filtered inputs. Defaults to identity if not provided.
   */
  LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, matrix_t costMatrix = matrix_t());

  /** Get the loopshaping type */
  LoopshapingType getType() const { return loopshapingType_; }

  /** True if all matrices of the loopshaping filter are diagonal */
  bool isDiagonal() const { return diagonal_; }

  /** Get access to the filter specification */
  const Filter& getInputFilter() const { return filter_; }

  /** Compute the cost on the filtered inputs */
  scalar_t loopshapingCost(const vector_t& filteredInput) const { return 0.5 * filteredInput.dot(R_ * filteredInput); }

  /** Get the quadratic cost matrix for the filtered inputs */
  matrix_t& costMatrix() { return R_; }

  /** Display details of the LoopshapingDefinition  */
  void print() const;

  /**
   * @param state : state of the augmented system
   * @return state of the original system
   */
  vector_t getSystemState(const vector_t& state) const { return state.head(state.rows() - filter_.getNumStates()); }
  void getSystemState(const vector_t& state, vector_t& systemState) const {
    systemState = state.head(state.rows() - filter_.getNumStates());
  }

  /**
   * @param state : state of the augmented system
   * @param input : input of the augmented system
   * @return input of the original system
   */
  vector_t getSystemInput(const vector_t& state, const vector_t& input) const;
  void getSystemInput(const vector_t& state, const vector_t& input, vector_t& systemInput) const;

  /**
   * @param state : state of the augmented system
   * @return state of the loopshaping filter
   */
  vector_t getFilterState(const vector_t& state) const { return state.tail(filter_.getNumStates()); }
  void getFilterState(const vector_t& state, vector_t& filterState) const { filterState = state.tail(filter_.getNumStates()); }

  /**
   * @param state : state of the augmented system
   * @param input : input of the augmented system
   * @return the filtered input
   */
  vector_t getFilteredInput(const vector_t& state, const vector_t& input) const;
  void getFilteredInput(const vector_t& state, const vector_t& input, vector_t& filteredInput) const;

  /**
   * @param filterState : state of the loopshaping filter
   * @param input : input of the augmented system
   * @return time derivative of the filter state
   */
  vector_t filterFlowMap(const vector_t& filterState, const vector_t& input) const;

  /**
   * @param systemState : state of the original system
   * @param filterState : state of the loopshaping filter
   * @return state of the augmented system
   */
  vector_t concatenateSystemAndFilterState(const vector_t& systemState, const vector_t& filterState) const;

  /**
   * @param systemInput : input of the original system
   * @param filterInput : the filtered input
   * @return input of the augmented system
   */
  vector_t augmentedSystemInput(const vector_t& systemInput, const vector_t& filterInput) const;

  /**
   * Finds a loopshaping state and input such that they are in equilibrium with a given system input
   * @param systemInput : input of the original system
   * @param filterState (return) : state of the loopshaping filter
   * @param filterInput (return) : the filtered input
   */
  void getFilterEquilibrium(const vector_t& systemInput, vector_t& filterState, vector_t& filterInput) const;

  /**
   * Finds a loopshaping input such that they are in equilibrium with a given system input
   * @param systemInput : input of the original system
   * @param filterState : state of the loopshaping filter
   * @param filterInput (return) : the filtered input
   */
  void getFilterEquilibriumGivenState(const vector_t& systemInput, const vector_t& filterState, vector_t& filterInput) const;

 private:
  Filter filter_;
  LoopshapingType loopshapingType_;
  bool diagonal_;
  matrix_t R_;
};

}  // namespace ocs2
