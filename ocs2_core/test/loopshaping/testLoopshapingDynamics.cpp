

#include "testLoopshapingDynamics.h"
#include <gtest/gtest.h>

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingDynamics, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateDynamics) {
  // Evaluate system
  vector_t dx_sys = this->testSystem->computeFlowMap(this->t, this->x_sys_, this->u_sys_);

  // Evaluate loopshaping system
  vector_t dx = this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_, this->u_);

  // System part of the flowmap should stay the same
  ASSERT_TRUE(dx_sys.isApprox(dx.head(dx_sys.rows())));
};

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateDynamicsDerivative) {
  // Extract linearization
  vector_t dx_0 = this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_, this->u_);
  matrix_t A = this->testLoopshapingDynamicsDerivative->getFlowMapDerivativeState();
  matrix_t B = this->testLoopshapingDynamicsDerivative->getFlowMapDerivativeInput();

  // Reevaluate at disturbed state
  vector_t dx_disturbance =
      this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);

  // Evaluate approximation
  vector_t dx_approximation = dx_0 + A * this->x_disturbance_ + B * this->u_disturbance_;

  // Difference between new evaluation and linearization should be less than tol
  ASSERT_LE((dx_disturbance - dx_approximation).array().abs().maxCoeff(), this->tol);
}