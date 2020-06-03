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

#include "LinearConstraintAD.h"

using namespace ocs2;

class testCppADCG_constraintFixture : public ::testing::Test {
 public:
  const size_t stateDim_ = 8;
  const size_t inputDim_ = 5;
  const size_t numStateInputConstraint_ = inputDim_ - 1;
  const size_t numStateOnlyConstraint_ = inputDim_ - 1;
  const size_t numStateOnlyFinalConstraint_ = inputDim_ - 1;
  const size_t maxNumConstraint_ = inputDim_;

  testCppADCG_constraintFixture() { create(); };

  void create() {
    vector_t e = vector_t::Random(numStateInputConstraint_);
    matrix_t C = matrix_t::Random(numStateInputConstraint_, stateDim_);
    matrix_t D = matrix_t::Random(numStateInputConstraint_, inputDim_);
    vector_t h = vector_t::Random(numStateOnlyConstraint_);
    matrix_t F = matrix_t::Random(numStateOnlyConstraint_, stateDim_);
    vector_t h_f = vector_t::Random(numStateOnlyFinalConstraint_);
    matrix_t F_f = matrix_t::Random(numStateOnlyFinalConstraint_, stateDim_);

    linearConstraint_.reset(new LinearConstraint(stateDim_, inputDim_, e, C, D, h, F, h_f, F_f));

    adLinearConstraint.reset(new LinearConstraintAD(stateDim_, inputDim_, e, C, D, h, F, h_f, F_f));

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

  vector_t x;
  vector_t u;

  for (size_t it = 0; it < numTests; it++) {
    x.setRandom(stateDim_);
    u.setRandom(inputDim_);

    constraint1->setCurrentStateAndControl(0.0, x, u);
    constraint2->setCurrentStateAndControl(0.0, x, u);

    vector_t g1 = constraint1->getStateInputEqualityConstraint();
    vector_t ad_g1 = constraint2->getStateInputEqualityConstraint();
    if (!g1.isApprox(ad_g1, precision)) {
      std::cout << "g1:    " << g1.transpose() << std::endl;
      std::cout << "ad_g1: " << ad_g1.transpose() << std::endl;
      success = false;
    }

    vector_t g2 = constraint1->getStateEqualityConstraint();
    vector_t ad_g2 = constraint2->getStateEqualityConstraint();
    if (!g2.isApprox(ad_g2, precision)) {
      std::cout << "g2:    " << g2.transpose() << std::endl;
      std::cout << "ad_g2: " << ad_g2.transpose() << std::endl;
      success = false;
    }

    vector_t g2f = constraint1->getFinalStateEqualityConstraint();
    vector_t ad_g2f = constraint2->getFinalStateEqualityConstraint();
    if (!g2f.isApprox(ad_g2f, precision)) {
      std::cout << "g2f:    " << g2f.transpose() << std::endl;
      std::cout << "ad_g2f: " << ad_g2f.transpose() << std::endl;
      success = false;
    }

    matrix_t C1 = constraint1->getStateInputEqualityConstraintDerivativesState();
    matrix_t ad_C1 = constraint2->getStateInputEqualityConstraintDerivativesState();
    if (!C1.isApprox(ad_C1, precision)) {
      std::cout << "C1:    " << C1.transpose() << std::endl;
      std::cout << "ad_C1: " << ad_C1.transpose() << std::endl;
      success = false;
    }

    matrix_t D1 = constraint1->getStateInputEqualityConstraintDerivativesInput();
    matrix_t ad_D1 = constraint2->getStateInputEqualityConstraintDerivativesInput();
    if (!D1.isApprox(ad_D1, precision)) {
      std::cout << "D1:    " << D1.transpose() << std::endl;
      std::cout << "ad_D1: " << ad_D1.transpose() << std::endl;
      success = false;
    }

    matrix_t F2 = constraint1->getStateEqualityConstraintDerivativesState();
    matrix_t ad_F2 = constraint2->getStateEqualityConstraintDerivativesState();
    if (!F2.isApprox(ad_F2, precision)) {
      std::cout << "F2:    " << F2.transpose() << std::endl;
      std::cout << "ad_F2: " << ad_F2.transpose() << std::endl;
      success = false;
    }

    matrix_t F2f = constraint1->getFinalStateEqualityConstraintDerivativesState();
    matrix_t ad_F2f = constraint2->getFinalStateEqualityConstraintDerivativesState();
    if (!F2f.isApprox(ad_F2f, precision)) {
      std::cout << "F2f:    " << F2f.transpose() << std::endl;
      std::cout << "ad_F2f: " << ad_F2f.transpose() << std::endl;
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
