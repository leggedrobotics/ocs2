//
// Created by ruben on 02.11.18.
//

#include <gtest/gtest.h>

#include "testLoopshapingConfigurations.h"

#include <ocs2_core/loopshaping/LoopshapingPropertyTree.h>

using namespace ocs2;

TEST(testLoopshapingDefinition, readingAllDefinitions) {
  for (const auto config : configNames) {
    const auto configPath = getAbsolutePathToConfigurationFile(config);
    auto loopshapingDefinition = loopshaping_property_tree::load(configPath);
    loopshapingDefinition->print();
  }

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, stateInputAccessFunctions) {
  for (const auto config : configNames) {
    const auto configPath = getAbsolutePathToConfigurationFile(config);
    auto loopshapingDefinition = loopshaping_property_tree::load(configPath);

    const size_t stateDim = loopshapingDefinition->getInputFilter().getNumStates();
    const size_t systemStateDim = 1;
    const size_t augmentedstateDim = stateDim + systemStateDim;
    const size_t inputDim = loopshapingDefinition->getInputFilter().getNumInputs();

    const vector_t augmentedState = vector_t::Random(augmentedstateDim);
    const vector_t augmentedInput = vector_t::Random(inputDim);

    const vector_t systemState = loopshapingDefinition->getSystemState(augmentedState);
    ASSERT_TRUE(augmentedState.head(systemStateDim).isApprox(systemState));

    const vector_t filterState = loopshapingDefinition->getFilterState(augmentedState);
    ASSERT_TRUE(augmentedState.tail(stateDim).isApprox(filterState));

    const vector_t augmentedStateConstructed = loopshapingDefinition->concatenateSystemAndFilterState(systemState, filterState);
    ASSERT_TRUE(augmentedState.isApprox(augmentedStateConstructed));

    const vector_t systemInput = loopshapingDefinition->getSystemInput(augmentedState, augmentedInput);
    const vector_t filteredInput = loopshapingDefinition->getFilteredInput(augmentedState, augmentedInput);
    const vector_t augmentedInputConstructed = loopshapingDefinition->augmentedSystemInput(systemInput, filteredInput);
    ASSERT_TRUE(augmentedInput.isApprox(augmentedInputConstructed));

    vector_t equilibriumState;
    vector_t equilibriumInput;
    loopshapingDefinition->getFilterEquilibrium(systemInput, equilibriumState, equilibriumInput);
  }
}
