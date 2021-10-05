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

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

LoopshapingTestConfiguration::LoopshapingTestConfiguration(const std::string& configName) {
  // Load loopshaping definition
  const std::string settingsFile = getAbsolutePathToConfigurationFile(configName);
  loopshapingDefinition_ = loopshaping_property_tree::load(settingsFile);

  systemStateDim_ = 1;  // Doesn't matter, just there to create a different size augmented state.
  filterStateDim_ = loopshapingDefinition_->getInputFilter().getNumStates();
  inputDim_ = loopshapingDefinition_->getInputFilter().getNumInputs();

  // Set up state and input
  t = 0.5;
  const scalar_t eps = 1e-2;
  getRandomStateInput(systemStateDim_, filterStateDim_, inputDim_, x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);
  getRandomStateInput(systemStateDim_, filterStateDim_, inputDim_, x_sys_disturbance_, u_sys_disturbance_, x_filter_disturbance_,
                      u_filter_disturbance_, x_disturbance_, u_disturbance_, eps);

  preComp_sys_.reset(new PreComputation);
  preComp_.reset(new LoopshapingPreComputation(*preComp_sys_, loopshapingDefinition_));
};

void LoopshapingTestConfiguration::getRandomStateInput(size_t systemStateDim, size_t filterStateDim, size_t inputDim, vector_t& x_sys,
                                                       vector_t& u_sys, vector_t& x_filter, vector_t& u_filter, vector_t& x, vector_t& u,
                                                       scalar_t range) {
  // Set random state
  x.setRandom(systemStateDim + filterStateDim);
  u.setRandom(inputDim);

  // Scale the randomness
  x *= range;
  u *= range;

  // Retreive system and filter state
  x_sys = loopshapingDefinition_->getSystemState(x);
  u_sys = loopshapingDefinition_->getSystemInput(x, u);
  x_filter = loopshapingDefinition_->getFilterState(x);
  u_filter = loopshapingDefinition_->getFilteredInput(x, u);
}

}  // namespace ocs2
