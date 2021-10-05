

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
