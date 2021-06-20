/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>

#include <ocs2_core/constraint/StateConstraintCollection.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>
#include "testConstraints.h"

TEST(TestConstraintCollection, add) {
  ocs2::StateInputConstraintCollection constraintCollection;

  // Add after construction
  std::unique_ptr<TestEmptyConstraint> constraintTerm(new TestEmptyConstraint());
  ASSERT_NO_THROW({ constraintCollection.add("Constraint1", std::move(constraintTerm)); });
}

TEST(TestConstraintCollection, numberOfConstraints) {
  ocs2::StateInputConstraintCollection constraintCollection;

  // Initially we have zero constraints for all types
  EXPECT_EQ(constraintCollection.getNumConstraints(0.0), 0);

  // Add Linear inequality constraint term, which has 2 constraints
  std::unique_ptr<TestDummyConstraint> constraintTerm(new TestDummyConstraint());
  const size_t addedConstraints = constraintTerm->getNumConstraints(0.0);
  constraintCollection.add("Constraint1", std::move(constraintTerm));

  // Check the right constraint size is incremented
  EXPECT_EQ(constraintCollection.getNumConstraints(0.0), addedConstraints);
}

TEST(TestConstraintCollection, activatingConstraints) {
  ocs2::StateInputConstraintCollection constraintCollection;

  // Initially we have zero constraints for all types
  EXPECT_EQ(constraintCollection.getNumConstraints(0.0), 0);

  // Add Linear inequality constraint term, which has 2 constraints
  std::unique_ptr<TestDummyConstraint> constraintTerm(new TestDummyConstraint());
  const size_t addedConstraints = constraintTerm->getNumConstraints(0.0);
  constraintCollection.add("Constraint1", std::move(constraintTerm));

  // Check the right constraint size is incremented
  EXPECT_EQ(constraintCollection.getNumConstraints(0.0), addedConstraints);

  constraintCollection.get<TestDummyConstraint>("Constraint1").setActivity(false);

  // Check the right constraint size after deactivating the constraint
  EXPECT_EQ(constraintCollection.getNumConstraints(0.0), 0);
}

TEST(TestConstraintCollection, clone) {
  ocs2::StateInputConstraintCollection constraintCollection;
  std::unique_ptr<TestDummyConstraint> constraintTerm(new TestDummyConstraint());
  const size_t addedConstraints = constraintTerm->getNumConstraints(0.0);
  constraintCollection.add("Constraint1", std::move(constraintTerm));

  // move construct
  std::unique_ptr<ocs2::StateInputConstraintCollection> newCollection(constraintCollection.clone());
  EXPECT_EQ(newCollection->getNumConstraints(0.0), addedConstraints);
}

TEST(TestConstraintCollection, getValue) {
  using collection_t = ocs2::StateInputConstraintCollection;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  ocs2::vector_t x(3);
  ocs2::vector_t u(2);
  u.setZero();
  x.setZero();

  // Zero constraints after creating
  auto constraintValues = constraintCollection.getValue(t, x, u, ocs2::PreComputation());
  EXPECT_EQ(constraintValues.size(), 0);

  // Add Linear inequality constraint term, which has 2 constraints, twice
  std::unique_ptr<TestDummyConstraint> constraintTerm1(new TestDummyConstraint());
  std::unique_ptr<TestDummyConstraint> constraintTerm2(new TestDummyConstraint());
  constraintCollection.add("Constraint1", std::move(constraintTerm1));
  constraintCollection.add("Constraint2", std::move(constraintTerm2));

  constraintValues = constraintCollection.getValue(t, x, u, ocs2::PreComputation());
  ASSERT_EQ(constraintValues.rows(), 4);
  EXPECT_EQ(constraintValues[0], 1.0);
  EXPECT_EQ(constraintValues[1], 2.0);
  EXPECT_EQ(constraintValues[2], 1.0);
  EXPECT_EQ(constraintValues[3], 2.0);
}

TEST(TestConstraintCollection, getLinearApproximation) {
  using collection_t = ocs2::StateInputConstraintCollection;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  ocs2::vector_t x(3);
  ocs2::vector_t u(2);
  u.setZero();
  x.setZero();

  // Add Linear inequality constraint term, which has 2 constraints, twice
  std::unique_ptr<TestDummyConstraint> constraintTerm1(new TestDummyConstraint());
  std::unique_ptr<TestDummyConstraint> constraintTerm2(new TestDummyConstraint());
  constraintCollection.add("Constraint1", std::move(constraintTerm1));
  constraintCollection.add("Constraint2", std::move(constraintTerm2));

  auto linearApproximation = constraintCollection.getLinearApproximation(t, x, u, ocs2::PreComputation());
  ASSERT_EQ(linearApproximation.f.size(), 4);
  EXPECT_EQ(linearApproximation.f(0), 1.0);
  EXPECT_EQ(linearApproximation.f(1), 2.0);
  EXPECT_EQ(linearApproximation.f(2), 1.0);
  EXPECT_EQ(linearApproximation.f(3), 2.0);
  EXPECT_EQ(linearApproximation.dfdx.row(0).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdx.row(1).sum(), 3);
  EXPECT_EQ(linearApproximation.dfdx.row(2).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdx.row(3).sum(), 3);
  EXPECT_EQ(linearApproximation.dfdu.row(0).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdu.row(1).sum(), 2);
  EXPECT_EQ(linearApproximation.dfdu.row(2).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdu.row(3).sum(), 2);
}

TEST(TestConstraintCollection, getQuadraticApproximation) {
  using collection_t = ocs2::StateInputConstraintCollection;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  ocs2::vector_t x(3);
  ocs2::vector_t u(2);
  u.setZero();
  x.setZero();

  // Add Linear inequality constraint term, which has 2 constraints, twice
  std::unique_ptr<TestDummyConstraint> constraintTerm1(new TestDummyConstraint());
  std::unique_ptr<TestDummyConstraint> constraintTerm2(new TestDummyConstraint());
  constraintCollection.add("Constraint1", std::move(constraintTerm1));
  constraintCollection.add("Constraint2", std::move(constraintTerm2));

  auto quadraticApproximation = constraintCollection.getQuadraticApproximation(t, x, u, ocs2::PreComputation());
  ASSERT_EQ(quadraticApproximation.f.size(), 4);
  EXPECT_EQ(quadraticApproximation.f(0), 1.0);
  EXPECT_EQ(quadraticApproximation.f(1), 2.0);
  EXPECT_EQ(quadraticApproximation.f(2), 1.0);
  EXPECT_EQ(quadraticApproximation.f(3), 2.0);
  EXPECT_EQ(quadraticApproximation.dfdx.row(0).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdx.row(1).sum(), 3);
  EXPECT_EQ(quadraticApproximation.dfdx.row(2).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdx.row(3).sum(), 3);
  EXPECT_EQ(quadraticApproximation.dfdu.row(0).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdu.row(1).sum(), 2);
  EXPECT_EQ(quadraticApproximation.dfdu.row(2).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdu.row(3).sum(), 2);
  EXPECT_EQ(quadraticApproximation.dfdxx[0].sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfduu[0].sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdux[0].sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdxx[1].sum(), 3 * 3);
  EXPECT_EQ(quadraticApproximation.dfduu[1].sum(), 2 * 2);
  EXPECT_EQ(quadraticApproximation.dfdux[1].sum(), 2 * 3);
}
