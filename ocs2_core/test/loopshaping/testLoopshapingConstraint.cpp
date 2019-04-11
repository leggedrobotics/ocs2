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


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}