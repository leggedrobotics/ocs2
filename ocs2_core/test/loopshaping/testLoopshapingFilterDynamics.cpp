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
#include <ocs2_core/loopshaping/LoopshapingPropertyTree.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingFilterDynamics.h>
#include "testLoopshapingConfigurations.h"

using namespace ocs2;

TEST(testLoopshapingFilterDynamics, Integration) {
  constexpr size_t FILTER_STATE_DIM = 3;
  constexpr size_t INPUT_DIM = 1;
  matrix_t A(FILTER_STATE_DIM, FILTER_STATE_DIM), B(FILTER_STATE_DIM, INPUT_DIM), C(INPUT_DIM, FILTER_STATE_DIM), D(INPUT_DIM, INPUT_DIM);
  A << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  B << 1.0, 1.0, 1.0;
  C << 0.0, 0.0, 0.0;
  D << 1.0;
  Filter filter(A, B, C, D);
  auto loopshapingDefinition = std::shared_ptr<LoopshapingDefinition>(new LoopshapingDefinition(LoopshapingType::outputpattern, filter));
  using TestLoopshapingFilterDynamics = LoopshapingFilterDynamics;
  TestLoopshapingFilterDynamics loopshapingFilterDynamics(loopshapingDefinition);

  // Initial conditions
  vector_t input, filter_state, filter_state0;
  input.setZero(INPUT_DIM);
  filter_state0.setOnes(FILTER_STATE_DIM);

  // Integrate
  scalar_t dt = 1.0;
  loopshapingFilterDynamics.setFilterState(filter_state0);
  loopshapingFilterDynamics.integrate(dt, input);
  filter_state = loopshapingFilterDynamics.getFilterState();

  for (int d = 0; d < FILTER_STATE_DIM; ++d) {
    // Check against analytical solution assuming diagonal A and zero input
    ASSERT_NEAR(filter_state(d), filter_state0(d) * exp(A(d, d) * dt), 1e-3);
  }
}
