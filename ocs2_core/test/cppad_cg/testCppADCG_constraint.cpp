/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
#include <boost/filesystem.hpp>
#include <iostream>

#include "LinearConstraintAD.h"
#include "ocs2_core/constraint/LinearConstraint.h"

class testCppADCG_constraintFixture : public ::testing::Test {
 public:
  static const size_t state_dim_ = 8;
  static const size_t input_dim_ = 5;

  using constraint_t = ocs2::ConstraintBase<state_dim_, input_dim_>;
  using linear_constraint_t = ocs2::LinearConstraint<state_dim_, input_dim_>;
  using ad_linear_constraint_t = ocs2::LinearConstraintAD<state_dim_, input_dim_>;

  using scalar_t = typename constraint_t::scalar_t;
  using state_vector_t = typename constraint_t::state_vector_t;
  using input_vector_t = typename constraint_t::input_vector_t;
  using state_matrix_t = typename constraint_t::state_matrix_t;
  using state_input_matrix_t = typename constraint_t::state_input_matrix_t;
  using constraint1_vector_t = typename constraint_t::constraint1_vector_t;
  using constraint2_vector_t = typename constraint_t::constraint2_vector_t;
  using constraint1_state_matrix_t = typename constraint_t::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename constraint_t::constraint1_input_matrix_t;
  using constraint2_state_matrix_t = typename constraint_t::constraint2_state_matrix_t;

  testCppADCG_constraintFixture() { create(); };

  void create() {
    size_t numStateInputConstraint = input_dim_ - 1;
    constraint1_vector_t e = constraint1_vector_t::Random();
    constraint1_state_matrix_t C = constraint1_state_matrix_t::Random();
    constraint1_input_matrix_t D = constraint1_input_matrix_t::Random();
    size_t numStateOnlyConstraint = input_dim_ - 1;
    constraint2_vector_t h = constraint2_vector_t::Random();
    constraint2_state_matrix_t F = constraint2_state_matrix_t::Random();
    size_t numStateOnlyFinalConstraint = input_dim_ - 1;
    constraint2_vector_t h_f = constraint2_vector_t::Random();
    constraint2_state_matrix_t F_f = constraint2_state_matrix_t::Random();

    linearConstraint_.reset(
        new linear_constraint_t(numStateInputConstraint, e, C, D, numStateOnlyConstraint, h, F, numStateOnlyFinalConstraint, h_f, F_f));

    adLinearConstraint.reset(
        new ad_linear_constraint_t(numStateInputConstraint, e, C, D, numStateOnlyConstraint, h, F, numStateOnlyFinalConstraint, h_f, F_f));

    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
    adLinearConstraint->initialize("testCppADCG_constraint", libraryFolder, true, true);
  }

  std::unique_ptr<linear_constraint_t> linearConstraint_;
  std::unique_ptr<ad_linear_constraint_t> adLinearConstraint;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
bool checkConstraints(const size_t numTests, ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>* const constraint1,
                      ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>* const constraint2, bool& success) {
  using constraint_t = ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename constraint_t::scalar_t;
  using state_vector_t = typename constraint_t::state_vector_t;
  using input_vector_t = typename constraint_t::input_vector_t;
  using state_matrix_t = typename constraint_t::state_matrix_t;
  using state_input_matrix_t = typename constraint_t::state_input_matrix_t;
  using constraint1_vector_t = typename constraint_t::constraint1_vector_t;
  using constraint2_vector_t = typename constraint_t::constraint2_vector_t;
  using constraint1_state_matrix_t = typename constraint_t::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename constraint_t::constraint1_input_matrix_t;
  using constraint2_state_matrix_t = typename constraint_t::constraint2_state_matrix_t;

  success = true;
  const scalar_t precision = 1e-9;

  state_vector_t x;
  input_vector_t u;

  for (size_t it = 0; it < numTests; it++) {
    x.setRandom();
    u.setRandom();

    constraint1->setCurrentStateAndControl(0.0, x, u);
    constraint2->setCurrentStateAndControl(0.0, x, u);

    size_t n1 = constraint1->numStateInputConstraint(0.0);
    size_t n2 = constraint1->numStateOnlyConstraint(0.0);
    size_t n2f = constraint1->numStateOnlyFinalConstraint(0.0);

    size_t ad_n1 = constraint2->numStateInputConstraint(0.0);
    size_t ad_n2 = constraint2->numStateOnlyConstraint(0.0);
    size_t ad_n2f = constraint2->numStateOnlyFinalConstraint(0.0);

    constraint1_vector_t g1, ad_g1;
    constraint1->getConstraint1(g1);
    constraint2->getConstraint1(ad_g1);
    if (!g1.head(n1).isApprox(ad_g1.head(ad_n1), precision)) {
      std::cout << "g1:    " << g1.head(n1).transpose() << std::endl;
      std::cout << "ad_g1: " << ad_g1.head(ad_n1).transpose() << std::endl;
      success = false;
    }

    constraint1_vector_t g2, ad_g2;
    constraint1->getConstraint2(g2);
    constraint2->getConstraint2(ad_g2);
    if (!g2.head(n2).isApprox(ad_g2.head(ad_n2), precision)) {
      std::cout << "g2:    " << g2.head(n2).transpose() << std::endl;
      std::cout << "ad_g2: " << ad_g2.head(ad_n2).transpose() << std::endl;
      success = false;
    }

    constraint1_vector_t g2f, ad_g2f;
    constraint1->getFinalConstraint2(g2f);
    constraint2->getFinalConstraint2(ad_g2f);
    if (!g2f.head(n2f).isApprox(ad_g2f.head(ad_n2f), precision)) {
      std::cout << "g2f:    " << g2f.head(n2f).transpose() << std::endl;
      std::cout << "ad_g2f: " << ad_g2f.head(ad_n2f).transpose() << std::endl;
      success = false;
    }

    constraint1_state_matrix_t C1, ad_C1;
    constraint1->getConstraint1DerivativesState(C1);
    constraint2->getConstraint1DerivativesState(ad_C1);
    if (!C1.topRows(n1).isApprox(ad_C1.topRows(ad_n1), precision)) {
      std::cout << "C1:    " << C1.topRows(n1).transpose() << std::endl;
      std::cout << "ad_C1: " << ad_C1.topRows(ad_n1).transpose() << std::endl;
      success = false;
    }

    constraint1_input_matrix_t D1, ad_D1;
    constraint1->getConstraint1DerivativesControl(D1);
    constraint2->getConstraint1DerivativesControl(ad_D1);
    if (!D1.topRows(n1).isApprox(ad_D1.topRows(ad_n1), precision)) {
      std::cout << "D1:    " << D1.topRows(n1).transpose() << std::endl;
      std::cout << "ad_D1: " << ad_D1.topRows(ad_n1).transpose() << std::endl;
      success = false;
    }

    constraint2_state_matrix_t F2, ad_F2;
    constraint1->getConstraint2DerivativesState(F2);
    constraint2->getConstraint2DerivativesState(ad_F2);
    if (!F2.topRows(n2).isApprox(ad_F2.topRows(ad_n2), precision)) {
      std::cout << "F2:    " << F2.topRows(n2).transpose() << std::endl;
      std::cout << "ad_F2: " << ad_F2.topRows(ad_n2).transpose() << std::endl;
      success = false;
    }

    constraint2_state_matrix_t F2f, ad_F2f;
    constraint1->getFinalConstraint2DerivativesState(F2f);
    constraint2->getFinalConstraint2DerivativesState(ad_F2f);
    if (!F2f.topRows(n2f).isApprox(ad_F2f.topRows(ad_n2f), precision)) {
      std::cout << "F2f:    " << F2f.topRows(n2f).transpose() << std::endl;
      std::cout << "ad_F2f: " << ad_F2f.topRows(ad_n2f).transpose() << std::endl;
      success = false;
    }

  }  // end of for loop
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_constraintFixture, constraint_test) {
  bool success;
  checkConstraints(100, linearConstraint_.get(), adLinearConstraint.get(), success);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_constraintFixture, clone_test) {
  constraint_t::Ptr ad_cloneConstraint(adLinearConstraint->clone());
  bool success;
  checkConstraints(100, linearConstraint_.get(), ad_cloneConstraint.get(), success);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_constraintFixture, multithread_test) {
  std::unique_ptr<constraint_t> cloneConstraint(linearConstraint_->clone());
  std::unique_ptr<constraint_t> ad_cloneConstraint(adLinearConstraint->clone());

  bool success = false;
  std::thread thread1(checkConstraints<state_dim_, input_dim_>, 10000, linearConstraint_.get(), adLinearConstraint.get(),
                      std::ref(success));

  bool successClone = false;
  std::thread thread2(checkConstraints<state_dim_, input_dim_>, 10000, cloneConstraint.get(), ad_cloneConstraint.get(),
                      std::ref(successClone));

  if (thread1.joinable()) {
    thread1.join();
  };
  if (thread2.joinable()) {
    thread2.join();
  };

  ASSERT_TRUE(success && successClone);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
