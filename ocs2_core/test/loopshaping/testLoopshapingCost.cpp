

#include "testLoopshapingCost.h"

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingCost, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingCost, testIntermediateCostApproximation) {
  // Extract Quadratic approximation
  this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_, this->u_);
  scalar_t L0 = this->testLoopshapingCost->getCost();
  vector_t dLdx = this->testLoopshapingCost->getCostDerivativeState();
  vector_t dLdu = this->testLoopshapingCost->getCostDerivativeInput();
  matrix_t ddLdxdx = this->testLoopshapingCost->getCostSecondDerivativeState();
  matrix_t ddLdudu = this->testLoopshapingCost->getCostSecondDerivativeInput();
  matrix_t ddLdudx = this->testLoopshapingCost->getCostDerivativeInputState();

  // Reevaluate at disturbed state
  this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  scalar_t L_disturbance = this->testLoopshapingCost->getCost();

  // Evaluate approximation
  scalar_t L_quad_approximation = L0 + dLdx.transpose() * this->x_disturbance_ + dLdu.transpose() * this->u_disturbance_ +
                                  0.5 * this->x_disturbance_.transpose() * ddLdxdx * this->x_disturbance_ +
                                  0.5 * this->u_disturbance_.transpose() * ddLdudu * this->u_disturbance_ +
                                  this->u_disturbance_.transpose() * ddLdudx * this->x_disturbance_;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingCost, testTerminalCostApproximation) {
  // Extract Quadratic approximation
  this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_, this->u_);
  scalar_t L0 = this->testLoopshapingCost->getTerminalCost();
  vector_t dLdx = this->testLoopshapingCost->getTerminalCostDerivativeState();
  matrix_t ddLdxdx = this->testLoopshapingCost->getTerminalCostSecondDerivativeState();

  // Reevaluate at disturbed state
  this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  scalar_t L_disturbance = this->testLoopshapingCost->getTerminalCost();

  // Evaluate approximation
  scalar_t L_quad_approximation =
      L0 + dLdx.transpose() * this->x_disturbance_ + 0.5 * this->x_disturbance_.transpose() * ddLdxdx * this->x_disturbance_;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
}
