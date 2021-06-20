

#include "testLoopshapingDynamics.h"
#include <gtest/gtest.h>

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingDynamics, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateDynamics) {
  // Evaluate system
  this->preComp_sys_->request(Request::Dynamics, this->t, this->x_, this->u_);
  vector_t dx_sys = this->testSystem->computeFlowMap(this->t, this->x_sys_, this->u_sys_, *this->preComp_sys_);

  // Evaluate loopshaping system
  this->preComp_->request(Request::Dynamics, this->t, this->x_, this->u_);
  vector_t dx = this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_, this->u_, *this->preComp_);

  // System part of the flowmap should stay the same
  ASSERT_TRUE(dx_sys.isApprox(dx.head(dx_sys.rows())));
};

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateDynamicsApproximation) {
  // Extract linearization
  this->preComp_->request(Request::Dynamics + Request::Approximation, this->t, this->x_, this->u_);
  const auto linearization = this->testLoopshapingDynamics->linearApproximation(this->t, this->x_, this->u_, *this->preComp_);

  // Reevaluate at disturbed state
  this->preComp_->request(Request::Dynamics, this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  vector_t dx_disturbance = this->testLoopshapingDynamics->computeFlowMap(this->t, this->x_ + this->x_disturbance_,
                                                                          this->u_ + this->u_disturbance_, *this->preComp_);

  // Evaluate approximation
  vector_t dx_approximation = linearization.f + linearization.dfdx * this->x_disturbance_ + linearization.dfdu * this->u_disturbance_;

  // Difference between new evaluation and linearization should be less than tol
  ASSERT_LE((dx_disturbance - dx_approximation).array().abs().maxCoeff(), this->tol);
}

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateJumpMap) {
  // Evaluate jump map
  this->preComp_sys_->requestPreJump(Request::Dynamics, this->t, this->x_sys_);
  const vector_t jumpMap_sys = this->testSystem->computeJumpMap(this->t, this->x_sys_, *this->preComp_sys_);

  this->preComp_->requestPreJump(Request::Dynamics, this->t, this->x_);
  const vector_t jumpMap = this->testLoopshapingDynamics->computeJumpMap(this->t, this->x_, *this->preComp_);

  EXPECT_TRUE(jumpMap.head(this->x_sys_.rows()).isApprox(jumpMap_sys));
}

TYPED_TEST(TestFixtureLoopShapingDynamics, evaluateJumpMapApproximation) {
  // Evaluate linearization
  this->preComp_sys_->requestPreJump(Request::Dynamics + Request::Approximation, this->t, this->x_sys_);
  const auto jumpMap_sys = this->testSystem->jumpMapLinearApproximation(this->t, this->x_sys_, *this->preComp_sys_);

  this->preComp_->requestPreJump(Request::Dynamics + Request::Approximation, this->t, this->x_);
  const auto jumpMap = this->testLoopshapingDynamics->jumpMapLinearApproximation(this->t, this->x_, *this->preComp_);

  EXPECT_TRUE(jumpMap.f.head(this->x_sys_.rows()).isApprox(jumpMap_sys.f));
  EXPECT_TRUE(jumpMap.dfdx.topLeftCorner(this->x_sys_.rows(), this->x_sys_.rows()).isApprox(jumpMap_sys.dfdx));
  EXPECT_TRUE(jumpMap.dfdu.size() == 0);
  EXPECT_TRUE(jumpMap_sys.dfdu.size() == 0);
}
