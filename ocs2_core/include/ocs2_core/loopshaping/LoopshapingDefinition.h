//
// Created by ruben on 26.10.18.
//

#ifndef OCS2_LOOPSHAPINGDEFINITION_H
#define OCS2_LOOPSHAPINGDEFINITION_H

#include <ocs2_core/loopshaping/LoopshapingFilter.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double gamma_;

  LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, double gamma = 0.9)
      : loopshapingType_(loopshapingType), filter_(std::move(filter)), gamma_(gamma) {}

  LoopshapingType getType() const { return loopshapingType_; };
  const Filter& getInputFilter() const { return filter_; };

  void print() const { filter_.print(); };

  template <typename DerivedStateVector, typename DerivedSystemState>
  void getSystemState(const DerivedStateVector& state, DerivedSystemState& systemState) {
    systemState = state.head(systemState.size());
  };

  template <typename DerivedStateVector, typename DerivedInputVector, typename DerivedSystemInput>
  void getSystemInput(const DerivedStateVector& state, const DerivedInputVector& input, DerivedSystemInput& systemInput) {
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

  template <typename DerivedStateVector, typename DerivedFilterState>
  void getFilterState(const DerivedStateVector& state, DerivedFilterState& filterState) {
    filterState = state.tail(filter_.getNumStates());
  };

  template <typename DerivedStateVector, typename DerivedInputVector, typename DerivedFilterInput>
  void getFilteredInput(const DerivedStateVector& state, const DerivedInputVector& input, DerivedFilterInput& filterInput) {
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

  template <typename DerivedSystemVector, typename DerivedFilterState, typename DerivedStateVector>
  void concatenateSystemAndFilterState(const DerivedSystemVector& systemState, const DerivedFilterState& filterState,
                                       DerivedStateVector& state) {
    state.head(systemState.size()) = systemState;
    state.tail(filter_.getNumStates()) = filterState;
  };

  template <typename DerivedSystemInput, typename DerivedFilterInput, typename DerivedInput>
  void concatenateSystemAndFilterInput(const DerivedSystemInput& systemInput, const DerivedFilterInput& filterInput, DerivedInput& input) {
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

  template <typename DerivedSystemInput, typename DerivedFilterState, typename DerivedFilterInput>
  void getFilterEquilibrium(const DerivedSystemInput& systemInput, DerivedFilterState& filterState, DerivedFilterInput& filterInput) {
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

#endif  // OCS2_LOOPSHAPINGDEFINITION_H
