//
// Created by rgrandia on 04.04.19.
//

#include <gtest/gtest.h>
#include "testLoopshapingDynamics.h"

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingDynamics, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateDynamics) {
  // Evaluate system
  typename TestFixtureLoopShapingDynamics<TypeParam>::system_state_vector_t dx_sys;
  this->testSystem->computeFlowMap(this->t, this->x_sys_, this->u_sys_, dx_sys);

  // Evaluate loopshaping system
  typename TestFixtureLoopShapingDynamics<TypeParam>::state_vector_t dx;
  this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_, this->u_, dx);

  // System part of the flowmap should stay the same
  ASSERT_TRUE(dx_sys.isApprox(dx.segment(0, this->SYSTEM_STATE_DIM)));
};

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateDynamicsDerivative) {
  // Test that dx = A delta_x + B delta_u + dx(x0, u0) is equal to dx(x0 + delta_x, u0 + delta_u);
  const double eps = 1e-6;
  const double tol = 1e-9;
  const int n_random_tests = 100;

  for (int i=0; i<n_random_tests; i++) {
    // Set random linearization point
    this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

    // Extract linearization
    typename TestFixtureLoopShapingDynamics<TypeParam>::state_vector_t dx_0;
    this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_, this->u_, dx_0);

    typename TestFixtureLoopShapingDynamics<TypeParam>::state_matrix_t A;
    typename TestFixtureLoopShapingDynamics<TypeParam>::state_input_matrix_t B;
    this->testLoopshapingDynamicsDerivative->getFlowMapDerivativeState(A);
    this->testLoopshapingDynamicsDerivative->getFlowMapDerivativeInput(B);

    // Pertubation
    typename TestFixtureLoopShapingDynamics<TypeParam>::state_vector_t x_disturbance;
    typename TestFixtureLoopShapingDynamics<TypeParam>::input_vector_t u_disturbance;
    typename TestFixtureLoopShapingDynamics<TypeParam>::system_state_vector_t x_sys_disturbance;
    typename TestFixtureLoopShapingDynamics<TypeParam>::system_input_vector_t u_sys_disturbance;
    typename TestFixtureLoopShapingDynamics<TypeParam>::filter_state_vector_t x_filter_disturbance;
    typename TestFixtureLoopShapingDynamics<TypeParam>::filter_input_vector_t u_filter_disturbance;
    this->getRandomStateInput(x_sys_disturbance,
                        u_sys_disturbance,
                        x_filter_disturbance,
                        u_filter_disturbance,
                        x_disturbance,
                        u_disturbance,
                        eps);

    // Reevaluate at disturbed state
    typename TestFixtureLoopShapingDynamics<TypeParam>::state_vector_t dx;
    this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_ + x_disturbance, this->u_ + u_disturbance, dx);

    // Difference between new evaluation and linearization should be less than tol
    ASSERT_LE((dx - (dx_0 + A * x_disturbance + B * u_disturbance)).array().abs().maxCoeff(), tol);
  }
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}