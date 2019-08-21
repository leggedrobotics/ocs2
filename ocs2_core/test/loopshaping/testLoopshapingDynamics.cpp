

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

  // Extract linearization
  typename TestFixtureLoopShapingDynamics<TypeParam>::state_vector_t dx_0;
  typename TestFixtureLoopShapingDynamics<TypeParam>::state_matrix_t A;
  typename TestFixtureLoopShapingDynamics<TypeParam>::state_input_matrix_t B;

  this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_, this->u_, dx_0);
  this->testLoopshapingDynamicsDerivative->getFlowMapDerivativeState(A);
  this->testLoopshapingDynamicsDerivative->getFlowMapDerivativeInput(B);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingDynamics<TypeParam>::state_vector_t dx_disturbance;
  this->testLoopshapingDynamics->computeFlowMap(this->t,
                                                this->x_ + this->x_disturbance_,
                                                this->u_ + this->u_disturbance_,
                                                dx_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingDynamics<TypeParam>::state_vector_t dx_approximation;
  dx_approximation = dx_0 + A * this->x_disturbance_ + B * this->u_disturbance_;

  // Difference between new evaluation and linearization should be less than tol
  ASSERT_LE((dx_disturbance - dx_approximation).array().abs().maxCoeff(), this->tol);
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}