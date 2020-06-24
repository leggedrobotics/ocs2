

#include "testLoopshapingConstraint.h"

const auto inf_ = std::numeric_limits<double>::infinity();

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingConstraint, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintEvaluation) {
  // Evaluate system
  vector_t g_system = this->testSystemConstraint->stateInputEqualityConstraint(this->t, this->x_sys_, this->u_sys_);
  const size_t numSystemConstraints = g_system.rows();

  // Evaluate loopshaping system
  vector_t g = this->testLoopshapingConstraint->stateInputEqualityConstraint(this->t, this->x_, this->u_);

  // System part of the constraints should stay the same
  ASSERT_LE((g_system.head(numSystemConstraints) - g.head(numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintApproximation) {
  // Extract approximation
  const auto g = this->testLoopshapingConstraint->stateInputEqualityConstraintLinearApproximation(this->t, this->x_, this->u_);

  // Reevaluate at disturbed state
  vector_t g_disturbance = this->testLoopshapingConstraint->stateInputEqualityConstraint(this->t, this->x_ + this->x_disturbance_,
                                                                                         this->u_ + this->u_disturbance_);

  // Evaluate approximation
  vector_t g_approximation = g.f + g.dfdx * this->x_disturbance_ + g.dfdu * this->u_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g_disturbance - g_approximation).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintEvaluation) {
  // Evaluate system
  vector_t g_system = this->testSystemConstraint->stateEqualityConstraint(this->t, this->x_sys_);
  const size_t numSystemConstraints = g_system.rows();

  // Evaluate loopshaping system
  vector_t g = this->testLoopshapingConstraint->stateEqualityConstraint(this->t, this->x_);

  // System part of the constraints should stay the same
  ASSERT_LE((g_system.head(numSystemConstraints) - g.head(numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintApproximation) {
  // Extract approximation
  const auto g = this->testLoopshapingConstraint->stateEqualityConstraintLinearApproximation(this->t, this->x_);

  // Reevaluate at disturbed state
  vector_t g_disturbance = this->testLoopshapingConstraint->stateEqualityConstraint(this->t, this->x_ + this->x_disturbance_);

  // Evaluate approximation
  vector_t g_approximation = g.f + g.dfdx * this->x_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g_disturbance - g_approximation).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintEvaluation) {
  // Evaluate system
  vector_t g_system = this->testSystemConstraint->finalStateEqualityConstraint(this->t, this->x_sys_);
  const size_t numSystemConstraints = g_system.rows();

  // Evaluate loopshaping system
  vector_t g = this->testLoopshapingConstraint->finalStateEqualityConstraint(this->t, this->x_);

  // System part of the constraints should stay the same
  ASSERT_LE((g_system.head(numSystemConstraints) - g.head(numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintApproximation) {
  // Extract approximation
  const auto g = this->testLoopshapingConstraint->finalStateEqualityConstraintLinearApproximation(this->t, this->x_);

  // Reevaluate at disturbed state
  vector_t g_disturbance = this->testLoopshapingConstraint->finalStateEqualityConstraint(this->t, this->x_ + this->x_disturbance_);

  // Evaluate approximation
  vector_t g_approximation = g.f + g.dfdx * this->x_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g_disturbance - g_approximation).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintEvaluation) {
  // Evaluate system
  vector_t h_system = this->testSystemConstraint->inequalityConstraint(this->t, this->x_sys_, this->u_sys_);

  // Evaluate loopshaping system
  vector_t h = this->testLoopshapingConstraint->inequalityConstraint(this->t, this->x_, this->u_);

  // System part of the constraints should stay the same
  ASSERT_LE((h_system - h.head(h_system.rows())).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintApproximation) {
  // Set random linearization point
  this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

  // Extract approximation
  const auto h = this->testLoopshapingConstraint->inequalityConstraintQuadraticApproximation(this->t, this->x_, this->u_);

  // Reevaluate at disturbed state
  vector_t h_disturbance =
      this->testLoopshapingConstraint->inequalityConstraint(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);

  // Evaluate approximation
  for (size_t i = 0; i < h.f.rows(); i++) {
    scalar_t h_approximation = h.f(i) + h.dfdx.row(i) * this->x_disturbance_ + h.dfdu.row(i) * this->u_disturbance_ +
                               0.5 * this->x_disturbance_.transpose() * h.dfdxx[i] * this->x_disturbance_ +
                               0.5 * this->u_disturbance_.transpose() * h.dfduu[i] * this->u_disturbance_ +
                               this->u_disturbance_.transpose() * h.dfdux[i] * this->x_disturbance_;
    ASSERT_LE(std::abs(h_disturbance[i] - h_approximation), this->tol);
  }
}