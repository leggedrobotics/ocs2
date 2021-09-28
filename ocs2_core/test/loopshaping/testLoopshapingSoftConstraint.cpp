

#include "testLoopshapingSoftConstraint.h"

#include <ocs2_core/test/testTools.h>

using namespace ocs2;

TYPED_TEST_CASE(TestFixtureLoopShapingSoftConstraint, FilterConfigurations);

TYPED_TEST(TestFixtureLoopShapingSoftConstraint, testStateInputValue) {
  this->preComputation->request(Request::Cost + Request::SoftConstraint, this->t, this->x, this->u);
  const auto L = this->loopshapingCost->getValue(this->t, this->x, this->u, this->targetTrajectories, *this->preComputation);
  const auto L_soft_constraint =
      this->loopshapingSoftConstraint->getValue(this->t, this->x, this->u, this->targetTrajectories, *this->preComputation);
  EXPECT_LE(std::abs(L - L_soft_constraint), this->tol);

  const auto L_system = this->systemCost->getValue(this->t, this->x_sys, this->u_sys, this->targetTrajectories, PreComputation());
  EXPECT_LE(std::abs(L_system - L_soft_constraint), this->tol);
}

TYPED_TEST(TestFixtureLoopShapingSoftConstraint, testStateInputApproximation) {
  this->preComputation->request(Request::Cost + Request::SoftConstraint + Request::Approximation, this->t, this->x, this->u);
  const auto L =
      this->loopshapingCost->getQuadraticApproximation(this->t, this->x, this->u, this->targetTrajectories, *this->preComputation);
  const auto L_soft_constraint = this->loopshapingSoftConstraint->getQuadraticApproximation(
      this->t, this->x, this->u, this->targetTrajectories, *this->preComputation);
  EXPECT_TRUE(isApprox(L, L_soft_constraint));
}

TYPED_TEST(TestFixtureLoopShapingSoftConstraint, testStateValue) {
  this->preComputation->requestFinal(Request::Cost + Request::SoftConstraint, this->t, this->x);
  const auto L = this->loopshapingStateCost->getValue(this->t, this->x, this->targetTrajectories, *this->preComputation);
  const auto L_soft_constraint =
      this->loopshapingStateSoftConstraint->getValue(this->t, this->x, this->targetTrajectories, *this->preComputation);
  EXPECT_LE(std::abs(L - L_soft_constraint), this->tol);

  const auto L_system = this->systemStateCost->getValue(this->t, this->x_sys, this->targetTrajectories, PreComputation());
  EXPECT_LE(std::abs(L_system - L_soft_constraint), this->tol);
}

TYPED_TEST(TestFixtureLoopShapingSoftConstraint, testStateApproximation) {
  this->preComputation->requestFinal(Request::Cost + Request::SoftConstraint + Request::Approximation, this->t, this->x);
  const auto L =
      this->loopshapingStateCost->getQuadraticApproximation(this->t, this->x, this->targetTrajectories, *this->preComputation);
  const auto L_soft_constraint = this->loopshapingStateSoftConstraint->getQuadraticApproximation(
      this->t, this->x, this->targetTrajectories, *this->preComputation);
  EXPECT_TRUE(isApprox(L, L_soft_constraint));
}
