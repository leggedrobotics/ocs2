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
  // Test that dx = A delta_x + B delta_u + dx(x0, u0) is equal to dx(x0 + delta_x, u0 + delta_u);
  const double eps = 1e-6;
  const double tol = 1e-9;
  const int n_random_tests = 100;

  for (int i=0; i<n_random_tests; i++) {
    // Set random linearization point
    getRandomStateInput(x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);

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

    // Difference between new evaluation and linearization should be less than tol
    ASSERT_LE((dx - (dx_0 + A * x_disturbance + B * u_disturbance)).array().abs().maxCoeff(), tol);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}