//
// Created by rgrandia on 03.04.19.
//

#include <gtest/gtest.h>

#include <ocs2_switched_model_interface/constraint/ConstraintCollection.h>
#include "testConstraintTerm.h"

TEST(TestConstraintCollection, add){
  ocs2::ConstraintCollection<3, 2> constraintCollection;

  // Add after construction
  std::unique_ptr<TestEmptyConstraint> constraintTerm(new TestEmptyConstraint());
  constraintCollection.add( std::move(constraintTerm), "Constraint1");
}

TEST(TestConstraintCollection, numberOfConstraints){
  ocs2::ConstraintCollection<3, 2> constraintCollection;

  // Initially we have zero constraints for all types
  ASSERT_EQ(constraintCollection.getConstraints().getNumConstraints(0.0), 0);

  // Add Linear inequality constraint term, which has 2 constraints
  std::unique_ptr<TestLinearConstraint> constraintTerm(new TestLinearConstraint());
  const size_t addedConstraints = constraintTerm->getNumConstraints(0.0);
  constraintCollection.add( std::move(constraintTerm), "Constraint1" );

  // Check the right constraint size is incremented
  ASSERT_EQ(constraintCollection.getConstraints().getNumConstraints(0.0), addedConstraints);
}

TEST(TestConstraintCollection, activatingConstraints){
  ocs2::ConstraintCollection<3, 2> constraintCollection;

  // Initially we have zero constraints for all types
  ASSERT_EQ(constraintCollection.getConstraints().getNumConstraints(0.0), 0);

  // Add Linear inequality constraint term, which has 2 constraints
  std::unique_ptr<TestLinearConstraint> constraintTerm(new TestLinearConstraint());
  const size_t addedConstraints = constraintTerm->getNumConstraints(0.0);
  constraintCollection.add( std::move(constraintTerm), "Constraint1" );

  // Check the right constraint size is incremented
  ASSERT_EQ(constraintCollection.getConstraints().getNumConstraints(0.0), addedConstraints);

  constraintCollection.modifyConstraint("Constraint1")->setActivity(false);

  // Check the right constraint size after deactivating the constraint
  ASSERT_EQ(constraintCollection.getConstraints().getNumConstraints(0.0), 0);
}

TEST(TestConstraintCollection, getValue_as_stdvector){
  using collection_t = ocs2::ConstraintCollection<3, 2>;
  collection_t constraintCollection;
  collection_t::ConstraintCollectionView_t::scalar_array_t constraintValues;

  // evaluation point
  double t = 0.0;
  collection_t::ConstraintTerm_t::input_vector_t u;
  collection_t::ConstraintTerm_t::state_vector_t x;
  u.setZero();
  x.setZero();

  // Zero constraints after creating
  constraintValues = constraintCollection.getConstraints().getValue(t, x, u);
  ASSERT_EQ(constraintValues.size(), 0);

  // Add Linear inequality constraint term, which has 2 constraints, twice
  std::unique_ptr<TestLinearConstraint> constraintTerm1(new TestLinearConstraint());
  std::unique_ptr<TestLinearConstraint> constraintTerm2(new TestLinearConstraint());
  constraintCollection.add( std::move(constraintTerm1), "Constraint1" );
  constraintCollection.add( std::move(constraintTerm2), "Constraint2" );

  constraintValues = constraintCollection.getConstraints().getValue(t, x, u);
  ASSERT_EQ(constraintValues.size(), 4);
  ASSERT_EQ(constraintValues[0], 1.0);
  ASSERT_EQ(constraintValues[1], 2.0);
  ASSERT_EQ(constraintValues[2], 1.0);
  ASSERT_EQ(constraintValues[3], 2.0);
}

TEST(TestConstraintCollection, getLinearApproximation){
  using collection_t = ocs2::ConstraintCollection<3, 2>;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  collection_t::ConstraintTerm_t::input_vector_t u;
  collection_t::ConstraintTerm_t::state_vector_t x;
  u.setZero();
  x.setZero();

  // Add Linear inequality constraint term, which has 2 constraints, twice
  std::unique_ptr<TestLinearConstraint> constraintTerm1(new TestLinearConstraint());
  std::unique_ptr<TestLinearConstraint> constraintTerm2(new TestLinearConstraint());
  constraintCollection.add( std::move(constraintTerm1), "Constraint1" );
  constraintCollection.add( std::move(constraintTerm2), "Constraint2" );

  auto linearApproximation = constraintCollection.getConstraints().getLinearApproximation(t, x, u);
  ASSERT_EQ(linearApproximation.constraintValues.size(), 4);
  ASSERT_EQ(linearApproximation.constraintValues[0], 1.0);
  ASSERT_EQ(linearApproximation.constraintValues[1], 2.0);
  ASSERT_EQ(linearApproximation.constraintValues[2], 1.0);
  ASSERT_EQ(linearApproximation.constraintValues[3], 2.0);
  ASSERT_EQ(linearApproximation.derivativeState[0].sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeState[1].sum(), 3);
  ASSERT_EQ(linearApproximation.derivativeState[2].sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeState[3].sum(), 3);
  ASSERT_EQ(linearApproximation.derivativeInput[0].sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeInput[1].sum(), 2);
  ASSERT_EQ(linearApproximation.derivativeInput[2].sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeInput[3].sum(), 2);
}

TEST(TestConstraintCollection, getQuadraticApproximation){
  using collection_t = ocs2::ConstraintCollection<3, 2>;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  collection_t::ConstraintTerm_t::input_vector_t u;
  collection_t::ConstraintTerm_t::state_vector_t x;
  u.setZero();
  x.setZero();

  // Add Linear inequality constraint term, which has 2 constraints, twice
  std::unique_ptr<TestLinearConstraint> constraintTerm1(new TestLinearConstraint());
  std::unique_ptr<TestLinearConstraint> constraintTerm2(new TestLinearConstraint());
  constraintCollection.add( std::move(constraintTerm1), "Constraint1" );
  constraintCollection.add( std::move(constraintTerm2), "Constraint2" );

  auto quadraticApproximation = constraintCollection.getConstraints().getQuadraticApproximation(t, x, u);
  ASSERT_EQ(quadraticApproximation.secondDerivativesState[0].sum(), 3*3);
  ASSERT_EQ(quadraticApproximation.secondDerivativesInput[0].sum(), 2*2);
  ASSERT_EQ(quadraticApproximation.derivativesInputState[0].sum(), 2*3);
  ASSERT_EQ(quadraticApproximation.secondDerivativesState[1].sum(), 3*3);
  ASSERT_EQ(quadraticApproximation.secondDerivativesInput[1].sum(), 2*2);
  ASSERT_EQ(quadraticApproximation.derivativesInputState[1].sum(), 2*3);
}

TEST(TestConstraintCollection, getValue_as_eigenvector){
  using collection_t = ocs2::ConstraintCollection<3, 2>;
  collection_t constraintCollection;
  Eigen::VectorXd constraintVector;

  // evaluation point
  double t = 0.0;
  collection_t::ConstraintTerm_t::input_vector_t u;
  collection_t::ConstraintTerm_t::state_vector_t x;
  u.setZero();
  x.setZero();

  // Zero constraints after creating
  constraintVector = constraintCollection.getConstraints().getValueAsVector(t, x, u);
  ASSERT_EQ(constraintVector.rows(), 0);

  // Add Linear inequality constraint term, which has 2 constraints, twice
  std::unique_ptr<TestLinearConstraint> constraintTerm1(new TestLinearConstraint());
  std::unique_ptr<TestLinearConstraint> constraintTerm2(new TestLinearConstraint());
  constraintCollection.add( std::move(constraintTerm1), "Constraint1" );
  constraintCollection.add( std::move(constraintTerm2), "Constraint2" );

  constraintVector = constraintCollection.getConstraints().getValueAsVector(t, x, u);
  ASSERT_EQ(constraintVector.rows(), 4);
  ASSERT_EQ(constraintVector[0], 1.0);
  ASSERT_EQ(constraintVector[1], 2.0);
  ASSERT_EQ(constraintVector[2], 1.0);
  ASSERT_EQ(constraintVector[3], 2.0);
}

TEST(TestConstraintCollection, getLinearApproximationAsMatrices){
  using collection_t = ocs2::ConstraintCollection<3, 2>;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  collection_t::ConstraintTerm_t::input_vector_t u;
  collection_t::ConstraintTerm_t::state_vector_t x;
  u.setZero();
  x.setZero();

  // Add one empty and one linear inequality constraint term, which has 2 constraints
  std::unique_ptr<TestEmptyConstraint> constraintTerm1(new TestEmptyConstraint());
  std::unique_ptr<TestLinearConstraint> constraintTerm2(new TestLinearConstraint());
  constraintCollection.add( std::move(constraintTerm1), "Constraint1" );
  constraintCollection.add( std::move(constraintTerm2), "Constraint2" );

  auto linearApproximation = constraintCollection.getConstraints().getLinearApproximationAsMatrices(t, x, u);
  ASSERT_EQ(linearApproximation.constraintValues[0], 1.0);
  ASSERT_EQ(linearApproximation.constraintValues[1], 2.0);
  ASSERT_EQ(linearApproximation.derivativeState.row(0).sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeState.row(1).sum(), 3);
  ASSERT_EQ(linearApproximation.derivativeInput.row(0).sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeInput.row(1).sum(), 2);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
