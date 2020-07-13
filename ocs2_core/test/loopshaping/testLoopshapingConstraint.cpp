

#include "testLoopshapingConstraint.h"

const auto inf_ = std::numeric_limits<double>::infinity();

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingConstraint, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintEvaluation) {
  // Evaluate system
  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  vector_t g_system = this->testSystemConstraint->getStateInputEqualityConstraint();
  const size_t numSystemConstraints = g_system.rows();

  // Evaluate loopshaping system
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  vector_t g = this->testLoopshapingConstraint->getStateInputEqualityConstraint();

  // System part of the constraints should stay the same
  ASSERT_LE((g_system.head(numSystemConstraints) - g.head(numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintApproximation) {
  // Extract approximation
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  vector_t g1_0 = this->testLoopshapingConstraint->getStateInputEqualityConstraint();
  matrix_t C = this->testLoopshapingConstraint->getStateInputEqualityConstraintDerivativesState();
  matrix_t D = this->testLoopshapingConstraint->getStateInputEqualityConstraintDerivativesInput();

  // Reevaluate at disturbed state
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  vector_t g1_disturbance = this->testLoopshapingConstraint->getStateInputEqualityConstraint();

  // Evaluate approximation
  vector_t g1_approximation = g1_0 + C * this->x_disturbance_ + D * this->u_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g1_disturbance - g1_approximation).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintEvaluation) {
  // Evaluate system
  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  vector_t g2_system = this->testSystemConstraint->getStateEqualityConstraint();
  const size_t numSystemConstraints = g2_system.rows();

  // Evaluate loopshaping system
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  vector_t g2 = this->testLoopshapingConstraint->getStateEqualityConstraint();

  // System part of the constraints should stay the same
  ASSERT_LE((g2_system.head(numSystemConstraints) - g2.head(numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintApproximation) {
  // Extract approximation
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  vector_t g2_0 = this->testLoopshapingConstraint->getStateEqualityConstraint();
  matrix_t C = this->testLoopshapingConstraint->getStateEqualityConstraintDerivativesState();

  // Reevaluate at disturbed state
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  vector_t g2_disturbance = this->testLoopshapingConstraint->getStateEqualityConstraint();

  // Evaluate approximation
  vector_t g2_approximation = g2_0 + C * this->x_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g2_disturbance - g2_approximation).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintEvaluation) {
  // Evaluate system
  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  vector_t g2_system = this->testSystemConstraint->getFinalStateEqualityConstraint();
  const size_t numSystemConstraints = g2_system.rows();

  // Evaluate loopshaping system
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  vector_t g2 = this->testLoopshapingConstraint->getFinalStateEqualityConstraint();

  // System part of the constraints should stay the same
  ASSERT_LE((g2_system.head(numSystemConstraints) - g2.head(numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintApproximation) {
  // Extract approximation
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  vector_t g2_0 = this->testLoopshapingConstraint->getFinalStateEqualityConstraint();
  matrix_t C = this->testLoopshapingConstraint->getFinalStateEqualityConstraintDerivativesState();

  // Reevaluate at disturbed state
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  vector_t g2_disturbance = this->testLoopshapingConstraint->getFinalStateEqualityConstraint();

  // Evaluate approximation
  vector_t g2_approximation = g2_0 + C * this->x_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g2_disturbance - g2_approximation).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintEvaluation) {
  // Evaluate system
  size_t numSystemConstraints;
  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  scalar_array_t h_system = this->testSystemConstraint->getInequalityConstraint();

  // Evaluate loopshaping system
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  scalar_array_t h = this->testLoopshapingConstraint->getInequalityConstraint();

  // System part of the constraints should stay the same
  for (size_t i = 0; i < numSystemConstraints; i++) {
    ASSERT_LE(std::abs(h_system[i] - h[i]), this->tol);
  }
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintApproximation) {
  // Set random linearization point
  this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

  // Extract approximation

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  scalar_array_t h = this->testLoopshapingConstraint->getInequalityConstraint();
  vector_array_t dhdx = this->testLoopshapingConstraint->getInequalityConstraintDerivativesState();
  vector_array_t dhdu = this->testLoopshapingConstraint->getInequalityConstraintDerivativesInput();
  matrix_array_t dhdxx = this->testLoopshapingConstraint->getInequalityConstraintSecondDerivativesState();
  matrix_array_t dhduu = this->testLoopshapingConstraint->getInequalityConstraintSecondDerivativesInput();
  matrix_array_t dhdux = this->testLoopshapingConstraint->getInequalityConstraintDerivativesInputState();

  // Reevaluate at disturbed state
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  scalar_array_t h_disturbance = this->testLoopshapingConstraint->getInequalityConstraint();

  // Evaluate approximation
  for (size_t i = 0; i < h.size(); i++) {
    scalar_t h_approximation = h[i] + dhdx[i].transpose() * this->x_disturbance_ + dhdu[i].transpose() * this->u_disturbance_ +
                               0.5 * this->x_disturbance_.transpose() * dhdxx[i] * this->x_disturbance_ +
                               0.5 * this->u_disturbance_.transpose() * dhduu[i] * this->u_disturbance_ +
                               this->u_disturbance_.transpose() * dhdux[i] * this->x_disturbance_;
    ASSERT_LE(std::abs(h_disturbance[i] - h_approximation), this->tol);
  }
}