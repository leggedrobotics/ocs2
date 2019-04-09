//
// Created by rgrandia on 04.04.19.
//

#include <gtest/gtest.h>
#include "testLoopshapingSystem.h"

using namespace ocs2;

TEST_F(TestLoopShapingDynamics_r_filter, evaluateDynamics) {
  // Evaluate system
  TestSystem::state_vector_t dx_sys;
  testSystem->computeFlowMap(t, x_sys_, u_sys_, dx_sys);

  // Evaluate loopshaping system
  TestLoopshapingDynamics::state_vector_t dx;
  testLoopshapingDynamics->computeFlowMap(t, x_, u_, dx);

  // System part of the flowmap should stay the same
  ASSERT_TRUE(dx_sys.isApprox(dx.segment(0, SYSTEM_STATE_DIM)));
}

TEST_F(TestLoopShapingDynamics_r_filter, evaluateDynamicsDerivative) {
  // Test that dx = A x + B u + dx_lin, and dx(x, u);
  const double eps = 1e-6;

  // Extract linearization
  TestLoopshapingDynamics::state_vector_t dx_0;
  testLoopshapingDynamics->computeFlowMap(t, x_, u_, dx_0);

  TestLoopshapingDynamicsDerivative::state_matrix_t A;
  TestLoopshapingDynamicsDerivative::state_input_matrix_t B;
  testLoopshapingDynamicsDerivative->getFlowMapDerivativeState(A);
  testLoopshapingDynamicsDerivative->getFlowMapDerivativeInput(B);

  // Pertubation
  state_vector_t x_disturbance;
  input_vector_t u_disturbance;
  system_state_vector_t x_sys_disturbance;
  system_input_vector_t u_sys_disturbance;
  filter_state_vector_t x_filter_disturbance;
  filter_input_vector_t u_filter_disturbance;
  getRandomStateInput(x_sys_disturbance,
                      u_sys_disturbance,
                      x_filter_disturbance,
                      u_filter_disturbance,
                      x_disturbance,
                      u_disturbance,
                      eps);

  // Reevaluate at disturbed state
  TestLoopshapingDynamics::state_vector_t dx;
  testLoopshapingDynamics->computeFlowMap(t, x_ + x_disturbance, u_ + u_disturbance, dx);

  // New evaluation should be equal to
  ASSERT_TRUE( (dx - dx_0).isApprox( A * x_disturbance + B * u_disturbance ) );
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}