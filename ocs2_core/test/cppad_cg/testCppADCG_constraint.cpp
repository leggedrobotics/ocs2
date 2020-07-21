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

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/LinearConstraint.h>

#include "../include/testTools.h"
#include "LinearConstraintAD.h"

using namespace ocs2;

class testCppADCG_constraintFixture : public ::testing::Test {
 public:
  const size_t stateDim_ = 8;
  const size_t inputDim_ = 5;
  const size_t numStateInputConstraint_ = inputDim_ - 1;
  const size_t numStateOnlyConstraint_ = inputDim_ - 1;
  const size_t numStateOnlyFinalConstraint_ = inputDim_ - 1;

  testCppADCG_constraintFixture() { create(); };

  void create() {
    vector_t e = vector_t::Random(numStateInputConstraint_);
    matrix_t C = matrix_t::Random(numStateInputConstraint_, stateDim_);
    matrix_t D = matrix_t::Random(numStateInputConstraint_, inputDim_);
    vector_t h = vector_t::Random(numStateOnlyConstraint_);
    matrix_t F = matrix_t::Random(numStateOnlyConstraint_, stateDim_);
    vector_t h_f = vector_t::Random(numStateOnlyFinalConstraint_);
    matrix_t F_f = matrix_t::Random(numStateOnlyFinalConstraint_, stateDim_);

    linearConstraint_.reset(new LinearConstraint(e, C, D, h, F, h_f, F_f));

    adLinearConstraint.reset(new LinearConstraintAD(e, C, D, h, F, h_f, F_f));

    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
    adLinearConstraint->initialize("testCppADCG_constraint", libraryFolder, true, true);
  }

  std::unique_ptr<LinearConstraint> linearConstraint_;
  std::unique_ptr<LinearConstraintAD> adLinearConstraint;
};

bool checkConstraints(const size_t numTests, ConstraintBase* const constraint1, ConstraintBase* const constraint2, bool& success,
                      size_t stateDim_, size_t inputDim_) {
  success = true;
  const scalar_t precision = 1e-9;
  const scalar_t t = 0.0;
  vector_t x;
  vector_t u;

  for (size_t it = 0; it < numTests && success; it++) {
    x.setRandom(stateDim_);
    u.setRandom(inputDim_);

    auto g1 = constraint1->stateInputEqualityConstraintLinearApproximation(t, x, u);
    auto g2 = constraint2->stateInputEqualityConstraintLinearApproximation(t, x, u);
    if (!isApprox(g1, g2)) {
      std::cout << "g1: \n" << g1 << std::endl;
      std::cout << "g2: \n" << g2 << std::endl;
      success = false;
    }

    auto h1 = constraint1->stateEqualityConstraintLinearApproximation(t, x);
    auto h2 = constraint2->stateEqualityConstraintLinearApproximation(t, x);
    if (!isApprox(h1, h2)) {
      std::cout << "h1: \n" << h1 << std::endl;
      std::cout << "h2: \n" << h2 << std::endl;
      success = false;
    }

    auto hf1 = constraint1->finalStateEqualityConstraintLinearApproximation(t, x);
    auto hf2 = constraint2->finalStateEqualityConstraintLinearApproximation(t, x);
    if (!isApprox(hf1, hf2)) {
      std::cout << "hf1: \n" << hf1 << std::endl;
      std::cout << "hf2: \n" << hf2 << std::endl;
      success = false;
    }
  }  // end of for loop

  return success;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_constraintFixture, constraint_test) {
  bool success;
  checkConstraints(100, linearConstraint_.get(), adLinearConstraint.get(), success, stateDim_, inputDim_);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_constraintFixture, clone_test) {
  std::unique_ptr<ConstraintBase> ad_cloneConstraint(adLinearConstraint->clone());
  bool success;
  checkConstraints(100, linearConstraint_.get(), ad_cloneConstraint.get(), success, stateDim_, inputDim_);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_constraintFixture, multithread_test) {
  std::unique_ptr<ConstraintBase> cloneConstraint(linearConstraint_->clone());
  std::unique_ptr<ConstraintBase> ad_cloneConstraint(adLinearConstraint->clone());

  bool success = false;
  std::thread thread1(checkConstraints, 10000, linearConstraint_.get(), adLinearConstraint.get(), std::ref(success), stateDim_, inputDim_);

  bool successClone = false;
  std::thread thread2(checkConstraints, 10000, cloneConstraint.get(), ad_cloneConstraint.get(), std::ref(successClone), stateDim_,
                      inputDim_);

  if (thread1.joinable()) {
    thread1.join();
  };
  if (thread2.joinable()) {
    thread2.join();
  };

  ASSERT_TRUE(success && successClone);
}
