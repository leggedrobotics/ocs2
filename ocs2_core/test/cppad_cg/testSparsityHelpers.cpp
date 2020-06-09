

#include <gtest/gtest.h>

#include "commonFixture.h"

#include <ocs2_core/automatic_differentiation/CppAdSparsity.h>

using namespace ocs2;

class SparsityFixture : public CommonCppAdParameterizedFixture {};

TEST_F(SparsityFixture, extractSparsity) {
  auto jacSparsity = cppad_sparsity::getJacobianSparsityPattern(*fun_);
  ASSERT_EQ(jacSparsity, jacobianSparsity_);

  auto hessSparsity = cppad_sparsity::getHessianSparsityPattern(*fun_);
  ASSERT_EQ(hessSparsity, hessianSparsity_);
}

TEST(CppAdSparsity, jacobianSparsity) {
  auto sparsity = cppad_sparsity::getJacobianVariableSparsity(1, 2);
  cppad_sparsity::SparsityPattern trueSparsity{{0, 1}};
  ASSERT_EQ(sparsity, trueSparsity);
}

TEST(CppAdSparsity, sparsityIntersection) {
  cppad_sparsity::SparsityPattern sparsity0{{0, 1, 2}, {0, 1, 2}, {0, 2}};
  cppad_sparsity::SparsityPattern sparsity1{{0, 1, 2}, {0, 2}, {0, 1, 2}};
  cppad_sparsity::SparsityPattern sparsity_intersection{{0, 1, 2}, {0, 2}, {0, 2}};

  // All intersections should reduce to the minimal case, i.e. sparsity_intersection
  ASSERT_EQ(sparsity_intersection, cppad_sparsity::getIntersection(sparsity0, sparsity1));
  ASSERT_EQ(sparsity_intersection, cppad_sparsity::getIntersection(sparsity_intersection, sparsity1));
  ASSERT_EQ(sparsity_intersection, cppad_sparsity::getIntersection(sparsity_intersection, sparsity0));
  ASSERT_EQ(sparsity_intersection, cppad_sparsity::getIntersection(sparsity_intersection, sparsity_intersection));
}

TEST(CppAdSparsity, hessianSparsity) {
  // H = [1 0; 0 0]
  auto sparsity = cppad_sparsity::getHessianVariableSparsity(1, 1);
  cppad_sparsity::SparsityPattern trueSparsity{{0}, {}};
  ASSERT_EQ(sparsity, trueSparsity);

  // H = [1 1 0; 0 1 0; 0 0 0]
  auto sparsityDiagonal = cppad_sparsity::getHessianVariableSparsity(2, 1);
  cppad_sparsity::SparsityPattern trueSparsityDiagonal{{0, 1}, {1}, {}};
  ASSERT_EQ(sparsityDiagonal, trueSparsityDiagonal);
}
