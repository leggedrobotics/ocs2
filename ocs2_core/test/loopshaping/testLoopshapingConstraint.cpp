//
// Created by rgrandia on 11.04.19.
//

#include "testLoopshapingConstraint.h"

const auto inf_ = std::numeric_limits<double>::infinity();

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingConstraint, FilterConfigurations
);

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintEvaluation) {
  const double tol = 1e-9;

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
  ASSERT_LE((g1_system.segment(0, numSystemConstraints) - g1.segment(numExtraLoopshapingConstraints, numSystemConstraints)).array().abs().maxCoeff(), tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintApproximation) {
  const double eps = 1e-2;
  const double tol = 1e-9;

  // Set random linearization point
  this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

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

  // Pertubation
  typename TestFixtureLoopShapingConstraint<TypeParam>::state_vector_t x_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::input_vector_t u_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_state_vector_t x_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_input_vector_t u_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_state_vector_t x_filter_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_input_vector_t u_filter_disturbance;
  this->getRandomStateInput(x_sys_disturbance,
                            u_sys_disturbance,
                            x_filter_disturbance,
                            u_filter_disturbance,
                            x_disturbance,
                            u_disturbance,
                            eps);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_vector_t g1_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + x_disturbance, this->u_ + u_disturbance);
  this->testLoopshapingConstraint->getConstraint1(g1_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint1_vector_t g1_approximation;
  g1_approximation = g1_0 + C * x_disturbance + D * u_disturbance;

  // System part of the constraints should stay the same
  ASSERT_LE((g1_disturbance.segment(0, numConstraints) - g1_approximation.segment(0, numConstraints)).array().abs().maxCoeff(), tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintEvaluation) {
  const double tol = 1e-9;

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
  ASSERT_LE((g2_system.segment(0, numSystemConstraints) - g2.segment(numExtraLoopshapingConstraints, numSystemConstraints)).array().abs().maxCoeff(), tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintApproximation) {
  const double eps = 1e-2;
  const double tol = 1e-9;

  // Set random linearization point
  this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

  // Extract approximation
  size_t numConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_0;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_state_matrix_t C;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numConstraints = this->testLoopshapingConstraint->numStateOnlyConstraint(this->t);
  this->testLoopshapingConstraint->getConstraint2(g2_0);
  this->testLoopshapingConstraint->getConstraint2DerivativesState(C);

  // Pertubation
  typename TestFixtureLoopShapingConstraint<TypeParam>::state_vector_t x_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::input_vector_t u_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_state_vector_t x_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_input_vector_t u_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_state_vector_t x_filter_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_input_vector_t u_filter_disturbance;
  this->getRandomStateInput(x_sys_disturbance,
                            u_sys_disturbance,
                            x_filter_disturbance,
                            u_filter_disturbance,
                            x_disturbance,
                            u_disturbance,
                            eps);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + x_disturbance, this->u_ + u_disturbance);
  this->testLoopshapingConstraint->getConstraint2(g2_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_approximation;
  g2_approximation = g2_0 + C * x_disturbance;

  // System part of the constraints should stay the same
  ASSERT_LE((g2_disturbance.segment(0, numConstraints) - g2_approximation.segment(0, numConstraints)).array().abs().maxCoeff(), tol);
};


TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintEvaluation) {
  const double tol = 1e-9;

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
  ASSERT_LE((g2_system.segment(0, numSystemConstraints) - g2.segment(numExtraLoopshapingConstraints, numSystemConstraints)).array().abs().maxCoeff(), tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testStateOnlyFinalConstraintApproximation) {
  const double eps = 1e-2;
  const double tol = 1e-9;

  // Set random linearization point
  this->getRandomStateInput(this->x_sys_, this->u_sys_, this->x_filter_, this->u_filter_, this->x_, this->u_);

  // Extract approximation
  size_t numConstraints;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_0;
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_state_matrix_t C;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_, this->u_);
  numConstraints = this->testLoopshapingConstraint->numStateOnlyConstraint(this->t);
  this->testLoopshapingConstraint->getFinalConstraint2(g2_0);
  this->testLoopshapingConstraint->getFinalConstraint2DerivativesState(C);

  // Pertubation
  typename TestFixtureLoopShapingConstraint<TypeParam>::state_vector_t x_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::input_vector_t u_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_state_vector_t x_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_input_vector_t u_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_state_vector_t x_filter_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_input_vector_t u_filter_disturbance;
  this->getRandomStateInput(x_sys_disturbance,
                            u_sys_disturbance,
                            x_filter_disturbance,
                            u_filter_disturbance,
                            x_disturbance,
                            u_disturbance,
                            eps);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + x_disturbance, this->u_ + u_disturbance);
  this->testLoopshapingConstraint->getFinalConstraint2(g2_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::constraint2_vector_t g2_approximation;
  g2_approximation = g2_0 + C * x_disturbance;

  // System part of the constraints should stay the same
  ASSERT_LE((g2_disturbance.segment(0, numConstraints) - g2_approximation.segment(0, numConstraints)).array().abs().maxCoeff(), tol);
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintEvaluation) {
  const double tol = 1e-9;

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
    ASSERT_LE(std::abs(h_system[i] - h[i]), tol);
  }
};

TYPED_TEST(TestFixtureLoopShapingConstraint, testInequalityConstraintApproximation) {
  const double eps = 1e-2;
  const double tol = 1e-9;

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

  // Pertubation
  typename TestFixtureLoopShapingConstraint<TypeParam>::state_vector_t x_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::input_vector_t u_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_state_vector_t x_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::system_input_vector_t u_sys_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_state_vector_t x_filter_disturbance;
  typename TestFixtureLoopShapingConstraint<TypeParam>::filter_input_vector_t u_filter_disturbance;
  this->getRandomStateInput(x_sys_disturbance,
                            u_sys_disturbance,
                            x_filter_disturbance,
                            u_filter_disturbance,
                            x_disturbance,
                            u_disturbance,
                            eps);

  // Reevaluate at disturbed state
  typename TestFixtureLoopShapingConstraint<TypeParam>::scalar_array_t h_disturbance;

  this->testLoopshapingConstraint->setCurrentStateAndControl(this->t, this->x_ + x_disturbance, this->u_ + u_disturbance);
  this->testLoopshapingConstraint->getInequalityConstraint(h_disturbance);

  // Evaluate approximation
  typename TestFixtureLoopShapingConstraint<TypeParam>::scalar_array_t h_approximation;
  for (size_t i=0; i<numConstraints; i++){
    h_approximation.push_back(h[i] + dhdx[i].transpose() * x_disturbance + dhdu[i].transpose() * u_disturbance + 0.5 * x_disturbance.transpose() * ddhdxdx[i] * x_disturbance
        + 0.5 * u_disturbance.transpose() * ddhdudu[i] * u_disturbance + u_disturbance.transpose() * ddhdudx[i] * x_disturbance);
    ASSERT_LE(std::abs(h_disturbance[i] - h_approximation[i]), tol);
  }
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}