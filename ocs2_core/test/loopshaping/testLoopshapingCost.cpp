

#include "testLoopshapingCost.h"

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingCost, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingCost, testIntermediateCostApproximation) {
  // Extract Quadratic approximation
  const auto L = this->testLoopshapingCost->costQuadraticApproximation(this->t, this->x_, this->u_);

  // Reevaluate at disturbed state
  scalar_t L_disturbance = this->testLoopshapingCost->cost(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);

  // Evaluate approximation
  scalar_t L_quad_approximation = L.f + L.dfdx.transpose() * this->x_disturbance_ + L.dfdu.transpose() * this->u_disturbance_ +
                                  0.5 * this->x_disturbance_.transpose() * L.dfdxx * this->x_disturbance_ +
                                  0.5 * this->u_disturbance_.transpose() * L.dfduu * this->u_disturbance_ +
                                  this->u_disturbance_.transpose() * L.dfdux * this->x_disturbance_;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingCost, testFinalCostApproximation) {
  // Extract Quadratic approximation
  const auto L = this->testLoopshapingCost->finalCostQuadraticApproximation(this->t, this->x_);

  // Reevaluate at disturbed state
  scalar_t L_disturbance = this->testLoopshapingCost->finalCost(this->t, this->x_ + this->x_disturbance_);

  // Evaluate approximation
  scalar_t L_quad_approximation =
      L.f + L.dfdx.transpose() * this->x_disturbance_ + 0.5 * this->x_disturbance_.transpose() * L.dfdxx * this->x_disturbance_;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
}
