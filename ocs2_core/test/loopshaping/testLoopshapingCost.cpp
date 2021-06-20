

#include "testLoopshapingCost.h"

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingCost, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingCost, testStateInputCostApproximation) {
  // Extract Quadratic approximation
  this->preComputation->request(Request::Cost + Request::Approximation, this->t, this->x, this->u);
  const auto L =
      this->loopshapingCost->getQuadraticApproximation(this->t, this->x, this->u, this->costDesiredTrajectories, *this->preComputation);

  // Reevaluate at disturbed state
  this->preComputation->request(Request::Cost, this->t, this->x + this->x_disturbance, this->u + this->u_disturbance);
  scalar_t L_disturbance = this->loopshapingCost->getValue(this->t, this->x + this->x_disturbance, this->u + this->u_disturbance,
                                                           this->costDesiredTrajectories, *this->preComputation);

  // Evaluate approximation
  scalar_t L_quad_approximation = L.f + L.dfdx.transpose() * this->x_disturbance + L.dfdu.transpose() * this->u_disturbance +
                                  0.5 * this->x_disturbance.transpose() * L.dfdxx * this->x_disturbance +
                                  0.5 * this->u_disturbance.transpose() * L.dfduu * this->u_disturbance +
                                  this->u_disturbance.transpose() * L.dfdux * this->x_disturbance;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingCost, testStateCostApproximation) {
  this->preComputation->requestFinal(Request::Cost + Request::Approximation, this->t, this->x);

  // Extract Quadratic approximation
  const auto L =
      this->loopshapingStateCost->getQuadraticApproximation(this->t, this->x, this->costDesiredTrajectories, *this->preComputation);

  // Reevaluate at disturbed state
  this->preComputation->requestFinal(Request::Cost, this->t, this->x + this->x_disturbance);
  scalar_t L_disturbance =
      this->loopshapingStateCost->getValue(this->t, this->x + this->x_disturbance, this->costDesiredTrajectories, *this->preComputation);

  // Evaluate approximation
  scalar_t L_quad_approximation =
      L.f + L.dfdx.transpose() * this->x_disturbance + 0.5 * this->x_disturbance.transpose() * L.dfdxx * this->x_disturbance;

  // Difference between new evaluation and approximation should be less than tol
  ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), this->tol);
}
