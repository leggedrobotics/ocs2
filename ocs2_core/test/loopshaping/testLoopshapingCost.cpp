//
// Created by rgrandia on 10.04.19.
//

#include "testLoopshapingCost.h"

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingCost, FilterConfigurations
);

TYPED_TEST(TestFixtureLoopShapingCost, testIntermediateCostApproximation) {
  // Test that quadratic approximation with loopshaping is consistent with a numdiff of the loopshaping cost
  const double eps = 1e-2;
  const double tol = 1e-9;
  const int n_random_tests = 10;

  for (int i = 0; i < n_random_tests; i++) {
    // Set random linearization point
    this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

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

    // Pertubation
    typename TestFixtureLoopShapingCost<TypeParam>::state_vector_t x_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::input_vector_t u_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::system_state_vector_t x_sys_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::system_input_vector_t u_sys_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::filter_state_vector_t x_filter_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::filter_input_vector_t u_filter_disturbance;
    this->getRandomStateInput(x_sys_disturbance,
                            u_sys_disturbance,
                            x_filter_disturbance,
                            u_filter_disturbance,
                            x_disturbance,
                            u_disturbance,
                            eps);

    // Reevaluate at disturbed state
    typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_disturbance;
    this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_ + x_disturbance, this->u_ + u_disturbance);
    this->testLoopshapingCost->getIntermediateCost(L_disturbance);

    // Evaluate approximation
    typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_quad_approximation;
    L_quad_approximation = L0 + dLdx.transpose() * x_disturbance + dLdu.transpose() * u_disturbance
        + 0.5*x_disturbance.transpose()*ddLdxdx*x_disturbance
        + 0.5*u_disturbance.transpose()*ddLdudu*u_disturbance
        + u_disturbance.transpose()*ddLdudx*x_disturbance;

      // Difference between new evaluation and approximation should be less than tol
      ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), tol);
  };
};

TYPED_TEST(TestFixtureLoopShapingCost, testTerminalCostApproximation) {
  // Test that quadratic approximation with loopshaping is consistent with a numdiff of the loopshaping cost
  const double eps = 1e-2;
  const double tol = 1e-9;
  const int n_random_tests = 10;

  for (int i = 0; i < n_random_tests; i++) {
    // Set random linearization point
    this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

    // Extract Quadratic approximation
    typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L0;
    typename TestFixtureLoopShapingCost<TypeParam>::state_vector_t dLdx;
    typename TestFixtureLoopShapingCost<TypeParam>::state_matrix_t ddLdxdx;
    this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_, this->u_);
    this->testLoopshapingCost->getTerminalCost(L0);
    this->testLoopshapingCost->getTerminalCostDerivativeState(dLdx);
    this->testLoopshapingCost->getTerminalCostSecondDerivativeState(ddLdxdx);

    // Pertubation
    typename TestFixtureLoopShapingCost<TypeParam>::state_vector_t x_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::input_vector_t u_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::system_state_vector_t x_sys_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::system_input_vector_t u_sys_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::filter_state_vector_t x_filter_disturbance;
    typename TestFixtureLoopShapingCost<TypeParam>::filter_input_vector_t u_filter_disturbance;
    this->getRandomStateInput(x_sys_disturbance,
                              u_sys_disturbance,
                              x_filter_disturbance,
                              u_filter_disturbance,
                              x_disturbance,
                              u_disturbance,
                              eps);

    // Reevaluate at disturbed state
    typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_disturbance;
    this->testLoopshapingCost->setCurrentStateAndControl(this->t, this->x_ + x_disturbance, this->u_ + u_disturbance);
    this->testLoopshapingCost->getTerminalCost(L_disturbance);

    // Evaluate approximation
    typename TestFixtureLoopShapingCost<TypeParam>::scalar_t L_quad_approximation;
    L_quad_approximation = L0 + dLdx.transpose() * x_disturbance + 0.5*x_disturbance.transpose()*ddLdxdx*x_disturbance;

    // Difference between new evaluation and approximation should be less than tol
    ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), tol);
  };
};



int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}