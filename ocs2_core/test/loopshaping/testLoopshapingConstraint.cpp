

#include "testLoopshapingConstraint.h"

#include <ocs2_core/loopshaping/constraint/LoopshapingFilterConstraint.h>

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingConstraint, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingConstraint, testFilterConstraint) {
  LoopshapingFilterConstraint filterConstraint(this->loopshapingDefinition);

  this->preComputation->request(Request::Constraint, this->t, this->x, this->u);
  const auto c = filterConstraint.getValue(this->t, this->x, this->u, *this->preComputation);
  EXPECT_TRUE(c.allFinite());

  this->preComputation->request(Request::Constraint + Request::Approximation, this->t, this->x, this->u);
  const auto clin = filterConstraint.getLinearApproximation(this->t, this->x, this->u, *this->preComputation);
  EXPECT_TRUE(clin.f.isApprox(c));
  EXPECT_TRUE(clin.f.allFinite());
  EXPECT_TRUE(clin.dfdx.allFinite());
  EXPECT_TRUE(clin.dfdu.allFinite());
}

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintEvaluation) {
  // Evaluate system
  vector_t g_system = this->systemConstraint->getValue(this->t, this->x_sys, this->u_sys, PreComputation());

  // Evaluate loopshaping system
  this->preComputation->request(Request::Constraint, this->t, this->x, this->u);
  vector_t g = this->loopshapingConstraint->getValue(this->t, this->x, this->u, *this->preComputation);

  // The constraint should stay the same
  EXPECT_LE((g_system - g).array().abs().maxCoeff(), this->tol);
}

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintLinearApproximation) {
  // Extract approximation
  this->preComputation->request(Request::Constraint + Request::Approximation, this->t, this->x, this->u);
  const auto g_linear = this->loopshapingConstraint->getLinearApproximation(this->t, this->x, this->u, *this->preComputation);
  const auto g_quadratic = this->loopshapingConstraint->getQuadraticApproximation(this->t, this->x, this->u, *this->preComputation);

  EXPECT_TRUE(g_linear.f.isApprox(g_quadratic.f));
  EXPECT_TRUE(g_linear.dfdx.isApprox(g_quadratic.dfdx));
  EXPECT_TRUE(g_linear.dfdu.isApprox(g_quadratic.dfdu));
}

TYPED_TEST(TestFixtureLoopShapingConstraint, DISABLED_testStateInputConstraintQuadraticApproximation) {
  // Extract approximation
  this->preComputation->request(Request::Constraint + Request::Approximation, this->t, this->x, this->u);
  const auto h = this->loopshapingConstraint->getQuadraticApproximation(this->t, this->x, this->u, *this->preComputation);

  // Reevaluate at disturbed state
  this->preComputation->request(Request::Constraint, this->t, this->x + this->x_disturbance, this->u + this->u_disturbance);
  vector_t h_disturbance =
      this->loopshapingConstraint->getValue(this->t, this->x + this->x_disturbance, this->u + this->u_disturbance, *this->preComputation);

  // Evaluate approximation
  for (size_t i = 0; i < h.f.rows(); i++) {
    scalar_t h_approximation = h.f(i) + h.dfdx.row(i) * this->x_disturbance + h.dfdu.row(i) * this->u_disturbance +
                               0.5 * this->x_disturbance.transpose() * h.dfdxx[i] * this->x_disturbance +
                               0.5 * this->u_disturbance.transpose() * h.dfduu[i] * this->u_disturbance +
                               this->u_disturbance.transpose() * h.dfdux[i] * this->x_disturbance;
    EXPECT_LE(std::abs(h_disturbance[i] - h_approximation), this->tol);
  }
}

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintEvaluation) {
  // Evaluate system
  vector_t g_system = this->systemStateConstraint->getValue(this->t, this->x_sys, PreComputation());

  // Evaluate loopshaping system
  this->preComputation->requestFinal(Request::Constraint, this->t, this->x);
  vector_t g = this->loopshapingStateConstraint->getValue(this->t, this->x, *this->preComputation);

  // System part of the constraints should stay the same
  EXPECT_LE((g_system - g).array().abs().maxCoeff(), this->tol);
}

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintLinearApproximation) {
  // Extract approximation
  this->preComputation->requestFinal(Request::Constraint + Request::Approximation, this->t, this->x);
  const auto g_linear = this->loopshapingStateConstraint->getLinearApproximation(this->t, this->x, *this->preComputation);
  const auto g_quadratic = this->loopshapingStateConstraint->getQuadraticApproximation(this->t, this->x, *this->preComputation);

  EXPECT_TRUE(g_linear.f.isApprox(g_quadratic.f));
  EXPECT_TRUE(g_linear.dfdx.isApprox(g_quadratic.dfdx));
}

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintQuadraticApproximation) {
  // Extract approximation
  this->preComputation->requestFinal(Request::Constraint + Request::Approximation, this->t, this->x);
  const auto h = this->loopshapingStateConstraint->getQuadraticApproximation(this->t, this->x, *this->preComputation);

  // Reevaluate at disturbed state
  this->preComputation->requestFinal(Request::Constraint, this->t, this->x + this->x_disturbance);
  vector_t h_disturbance = this->loopshapingStateConstraint->getValue(this->t, this->x + this->x_disturbance, *this->preComputation);

  // Evaluate approximation
  for (size_t i = 0; i < h.f.rows(); i++) {
    scalar_t h_approximation =
        h.f(i) + h.dfdx.row(i) * this->x_disturbance + 0.5 * this->x_disturbance.transpose() * h.dfdxx[i] * this->x_disturbance;
    EXPECT_LE(std::abs(h_disturbance[i] - h_approximation), this->tol);
  }
}
