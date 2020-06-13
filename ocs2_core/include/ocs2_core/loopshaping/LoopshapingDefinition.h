//
// Created by ruben on 26.10.18.
//

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
  scalar_t gamma_;

  LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, scalar_t gamma = 0.9)
      : loopshapingType_(loopshapingType), filter_(std::move(filter)), gamma_(gamma) {}

  LoopshapingType getType() const { return loopshapingType_; };
  const Filter& getInputFilter() const { return filter_; };

  void print() const { filter_.print(); };

  void getSystemState(const vector_t& state, vector_t& systemState) { systemState = state.head(systemState.size()); };

  void getSystemInput(const vector_t& state, const vector_t& input, vector_t& systemInput) {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern:
      case LoopshapingType::inputpattern:
        systemInput = input.head(systemInput.size());
        break;
      case LoopshapingType::eliminatepattern:
        // u = C*x + D*v
        systemInput.noalias() = filter_.getC() * state.tail(filter_.getNumStates());
        systemInput.noalias() += filter_.getD() * input;
        break;
    }
  };

  void getFilterState(const vector_t& state, vector_t& filterState) { filterState = state.tail(filter_.getNumStates()); };

  void getFilteredInput(const vector_t& state, const vector_t& input, vector_t& filterInput) {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern:
        filterInput = filter_.getC() * state.tail(filter_.getNumStates()) + filter_.getD() * input.head(filter_.getNumInputs());
        break;
      case LoopshapingType::inputpattern:
      case LoopshapingType::eliminatepattern:
        filterInput = input.tail(filter_.getNumInputs());
        break;
    }
  };

  void concatenateSystemAndFilterState(const vector_t& systemState, const vector_t& filterState, vector_t& state) {
    state.head(systemState.size()) = systemState;
    state.tail(filter_.getNumStates()) = filterState;
  };

  void concatenateSystemAndFilterInput(const vector_t& systemInput, const vector_t& filterInput, vector_t& input) {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern:
        input.head(systemInput.size()) = systemInput;
        break;
      case LoopshapingType::inputpattern:
        input.head(systemInput.size()) = systemInput;
        input.segment(systemInput.size(), filter_.getNumInputs()) = filterInput;
        break;
      case LoopshapingType::eliminatepattern:
        input.head(filter_.getNumInputs()) = filterInput;
        break;
    }
  };

  void getFilterEquilibrium(const vector_t& systemInput, vector_t& filterState, vector_t& filterInput) {
    switch (loopshapingType_) {
      case LoopshapingType::outputpattern:
        // When systemInput is the input to the filter
        filter_.findEquilibriumForInput(systemInput, filterState, filterInput);
        break;
      case LoopshapingType::inputpattern:
      case LoopshapingType::eliminatepattern:
        // When systemInput is the output of the filter
        filter_.findEquilibriumForOutput(systemInput, filterState, filterInput);
        break;
    }
  }

 private:
  Filter filter_;
  LoopshapingType loopshapingType_;
};

}  // namespace ocs2
