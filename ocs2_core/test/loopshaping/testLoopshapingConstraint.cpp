

#include "testLoopshapingConstraint.h"

const auto inf_ = std::numeric_limits<double>::infinity();

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingConstraint, FilterConfigurations
);

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintEvaluation) {
  // Evaluate system
  size_t numSystemConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_constraint1_vector_t g1_system;
  g1_system.setConstant(inf_);

  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  numSystemConstraints = this->testSystemConstraint->numStateInputConstraint(this->t);
  this->testSystemConstraint->getConstraint1(g1_system);

  // Evaluate loopshaping system
  size_t numTotalConstraints;
  size_t numExtraLoopshapingConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_vector_t g1;
  g1.setConstant(inf_);

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numTotalConstraints = this->testLoopshapingConstraint->numStateInputConstraint(this->t);
  numExtraLoopshapingConstraints = numTotalConstraints - numSystemConstraints;
  this->testLoopshapingConstraint->getConstraint1(g1);

  // System part of the constraints should stay the same
  ASSERT_LE((g1_system.segment(0, numSystemConstraints) - g1.segment(0, numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintApproximation) {
  // Extract approximation
  size_t numConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_vector_t g1_0;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_state_matrix_t C;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_input_matrix_t D;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numConstraints = this->testLoopshapingConstraint->numStateInputConstraint(this->t);
  this->testLoopshapingConstraint->getConstraint1(g1_0);
  this->testLoopshapingConstraint->getConstraint1DerivativesState(C);
  this->testLoopshapingConstraint->getConstraint1DerivativesControl(D);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_vector_t g1_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  this->testLoopshapingConstraint->getConstraint1(g1_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_vector_t g1_approximation;
  g1_approximation = g1_0 + C * this->x_disturbance_ + D * this->u_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g1_disturbance.segment(0, numConstraints) - g1_approximation.segment(0, numConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintEvaluation) {
  // Evaluate system
  size_t numSystemConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_constraint2_vector_t g2_system;
  g2_system.setConstant(inf_);

  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  numSystemConstraints = this->testSystemConstraint->numStateOnlyConstraint(this->t);
  this->testSystemConstraint->getConstraint2(g2_system);

  // Evaluate loopshaping system
  size_t numTotalConstraints;
  size_t numExtraLoopshapingConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2;
  g2.setConstant(inf_);

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numTotalConstraints = this->testLoopshapingConstraint->numStateOnlyConstraint(this->t);
  numExtraLoopshapingConstraints = numTotalConstraints - numSystemConstraints;
  this->testLoopshapingConstraint->getConstraint2(g2);

  // System part of the constraints should stay the same
  ASSERT_LE((g2_system.segment(0, numSystemConstraints) - g2.segment(numExtraLoopshapingConstraints, numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintApproximation) {
  // Extract approximation
  size_t numConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_0;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_state_matrix_t C;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numConstraints = this->testLoopshapingConstraint->numStateOnlyConstraint(this->t);
  this->testLoopshapingConstraint->getConstraint2(g2_0);
  this->testLoopshapingConstraint->getConstraint2DerivativesState(C);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  this->testLoopshapingConstraint->getConstraint2(g2_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_approximation;
  g2_approximation = g2_0 + C * this->x_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g2_disturbance.segment(0, numConstraints) - g2_approximation.segment(0, numConstraints)).array().abs().maxCoeff(), this->tol);
};


TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintEvaluation) {
  // Evaluate system
  size_t numSystemConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_constraint2_vector_t g2_system;
  g2_system.setConstant(inf_);
  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  numSystemConstraints = this->testSystemConstraint->numStateOnlyFinalConstraint(this->t);
  this->testSystemConstraint->getFinalConstraint2(g2_system);

  // Evaluate loopshaping system
  size_t numTotalConstraints;
  size_t numExtraLoopshapingConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2;
  g2.setConstant(inf_);

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numTotalConstraints = this->testLoopshapingConstraint->numStateOnlyFinalConstraint(this->t);
  numExtraLoopshapingConstraints = numTotalConstraints - numSystemConstraints;
  this->testLoopshapingConstraint->getFinalConstraint2(g2);

  // System part of the constraints should stay the same
  ASSERT_LE((g2_system.segment(0, numSystemConstraints) - g2.segment(numExtraLoopshapingConstraints, numSystemConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintApproximation) {
  // Extract approximation
  size_t numConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_0;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_state_matrix_t C;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numConstraints = this->testLoopshapingConstraint->numStateOnlyConstraint(this->t);
  this->testLoopshapingConstraint->getFinalConstraint2(g2_0);
  this->testLoopshapingConstraint->getFinalConstraint2DerivativesState(C);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  this->testLoopshapingConstraint->getFinalConstraint2(g2_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_approximation;
  g2_approximation = g2_0 + C * this->x_disturbance_;

  // System part of the constraints should stay the same
  ASSERT_LE((g2_disturbance.segment(0, numConstraints) - g2_approximation.segment(0, numConstraints)).array().abs().maxCoeff(), this->tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintEvaluation) {
  // Evaluate system
  size_t numSystemConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_scalar_array_t h_system;
  this->testSystemConstraint->setCurrentStateAndControl(this->t, this->x_sys_, this->u_sys_);
  numSystemConstraints = this->testSystemConstraint->numInequalityConstraint(this->t);
  this->testSystemConstraint->getInequalityConstraint(h_system);

  // Evaluate loopshaping system
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_scalar_array_t h;
  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  this->testLoopshapingConstraint->getInequalityConstraint(h);

  // System part of the constraints should stay the same
  for (size_t i=0; i<numSystemConstraints; i++){
    ASSERT_LE(std::abs(h_system[i] - h[i]), this->tol);
  }
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintApproximation) {
  // Set random linearization point
  this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

  // Extract approximation
  size_t numConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::scalar_array_t h;
  typename TestFixtureLoopShapingConstraint<TypeParam>::state_vector_array_t dhdx;
  typename TestFixtureLoopShapingConstraint<TypeParam>::input_vector_array_t dhdu;
  typename TestFixtureLoopShapingConstraint<TypeParam>::state_matrix_array_t ddhdxdx;
  typename TestFixtureLoopShapingConstraint<TypeParam>::input_matrix_array_t ddhdudu;
  typename TestFixtureLoopShapingConstraint<TypeParam>::input_state_matrix_array_t ddhdudx;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numConstraints = this->testLoopshapingConstraint->numInequalityConstraint(this->t);
  this->testLoopshapingConstraint->getInequalityConstraint(h);
  this->testLoopshapingConstraint->getInequalityConstraintDerivativesState(dhdx);
  this->testLoopshapingConstraint->getInequalityConstraintDerivativesInput(dhdu);
  this->testLoopshapingConstraint->getInequalityConstraintSecondDerivativesState(ddhdxdx);
  this->testLoopshapingConstraint->getInequalityConstraintSecondDerivativesInput(ddhdudu);
  this->testLoopshapingConstraint->getInequalityConstraintDerivativesInputState(ddhdudx);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::scalar_array_t h_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + this->x_disturbance_, this->u_ + this->u_disturbance_);
  this->testLoopshapingConstraint->getInequalityConstraint(h_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::scalar_array_t h_approximation;
  for (size_t i=0; i<numConstraints; i++){
    h_approximation.push_back(h[i] + dhdx[i].transpose() * this->x_disturbance_ + dhdu[i].transpose() * this->u_disturbance_ + 0.5 * this->x_disturbance_.transpose() * ddhdxdx[i] * this->x_disturbance_
        + 0.5 * this->u_disturbance_.transpose() * ddhdudu[i] * this->u_disturbance_ + this->u_disturbance_.transpose() * ddhdudx[i] * this->x_disturbance_);
    ASSERT_LE(std::abs(h_disturbance[i] - h_approximation[i]), this->tol);
  }
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}