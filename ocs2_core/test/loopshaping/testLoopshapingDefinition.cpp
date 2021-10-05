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
