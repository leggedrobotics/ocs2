

#include "testLoopshapingCost.h"

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingCost, FilterConfigurations
);

TYPED_TEST(TestFixtureLoopShapingCost, testIntermediateCostApproximation) {
  // Extract Quadratic approximation
  typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L0;
  typename TestFixtureLoopShapingCost<TypeParam>::state_vector_t dLdx;
  typename TestFixtureLoopShapingCost<TypeParam>::input_vector_t dLdu;
  typename TestFixtureLoopShapingCost<TypeParam>::state_matrix_t ddLdxdx;
  typename TestFixtureLoopShapingCost<TypeParam>::input_matrix_t ddLdudu;
  typename TestFixtureLoopShapingCost<TypeParam>::input_state_matrix_t ddLdudx;
  this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_, this->u_);
  this->testLoopshapingCost->getIntermediateCost(L0);
  this->testLoopshapingCost->getIntermediateCostDerivativeState(dLdx);
  this->testLoopshapingCost->getIntermediateCostDerivativeInput(dLdu);
  this->testLoopshapingCost->getIntermediateCostSecondDerivativeState(ddLdxdx);
  this->testLoopshapingCost->getIntermediateCostSecondDerivativeInput(ddLdudu);
  this->testLoopshapingCost->getIntermediateCostDerivativeInputState(ddLdudx);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_disturbance;
  this->testLoopshapingCost->setCurrentStateAndControl(this->t,
                                                       this->x_ + this->x_disturbance_,
                                                       this->u_ + this->u_disturbance_);
  this->testLoopshapingCost->getIntermediateCost(L_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_quad_approximation;
  L_quad_approximation = L0 + dLdx.transpose() * this->x_disturbance_ + dLdu.transpose() * this->u_disturbance_
      + 0.5 * this->x_disturbance_.transpose() * ddLdxdx * this->x_disturbance_
      + 0.5 * this->u_disturbance_.transpose() * ddLdudu * this->u_disturbance_
      + this->u_disturbance_.transpose() * ddLdudx * this->x_disturbance_;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingCost, testTerminalCostApproximation) {
  // Extract Quadratic approximation
  typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L0;
  typename TestFixtureLoopShapingCost<TypeParam>::state_vector_t dLdx;
  typename TestFixtureLoopShapingCost<TypeParam>::state_matrix_t ddLdxdx;
  this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_, this->u_);
  this->testLoopshapingCost->getTerminalCost(L0);
  this->testLoopshapingCost->getTerminalCostDerivativeState(dLdx);
  this->testLoopshapingCost->getTerminalCostSecondDerivativeState(ddLdxdx);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_disturbance;
  this->testLoopshapingCost->setCurrentStateAndControl(this->t,
                                                       this->x_ + this->x_disturbance_,
                                                       this->u_ + this->u_disturbance_);
  this->testLoopshapingCost->getTerminalCost(L_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_quad_approximation;
  L_quad_approximation = L0 + dLdx.transpose() * this->x_disturbance_
      + 0.5 * this->x_disturbance_.transpose() * ddLdxdx * this->x_disturbance_;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}