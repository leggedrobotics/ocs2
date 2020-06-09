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
#include <functional>
#include <iostream>

#include "LinearSystemDynamicsAD.h"
#include "ocs2_core/dynamics/LinearSystemDynamics.h"

using namespace ocs2;

class testCppADCG_dynamicsFixture : public ::testing::Test {
 public:
  const size_t stateDim_ = 4;
  const size_t inputDim_ = 2;

  testCppADCG_dynamicsFixture() { create(); };

  void create() {
    // Define dynamics parameters
    matrix_t A = matrix_t::Random(stateDim_, stateDim_);
    matrix_t B = matrix_t::Random(stateDim_, inputDim_);
    matrix_t G = matrix_t::Random(stateDim_, stateDim_);

    linearSystem_.reset(new LinearSystemDynamics(A, B, G));

    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
    adLinearSystem_.reset(new LinearSystemDynamicsAD(A, B, G));

    adLinearSystem_->initialize("testCppADCG_dynamics", libraryFolder, true, true);
  }

  std::unique_ptr<LinearSystemDynamics> linearSystem_;
  std::unique_ptr<LinearSystemDynamicsAD> adLinearSystem_;
};

/******************************************************************************/
/******************************************************************************/
/***************************************************************************** */
void checkSystemDynamics(const size_t numTests, DerivativesBase* const linearSystem1, DerivativesBase* const linearSystem2, bool& success,
                         size_t stateDim_, size_t inputDim_) {
  success = true;

  const scalar_t precision = 1e-9;

  vector_t x;
  vector_t u;

  for (size_t it = 0; it < numTests; it++) {
    x.setRandom(stateDim_);
    u.setRandom(inputDim_);

    linearSystem1->setCurrentStateAndControl(0.0, x, u);
    linearSystem2->setCurrentStateAndControl(0.0, x, u);

    matrix_t A, ad_A;
    linearSystem1->getFlowMapDerivativeState(A);
    linearSystem2->getFlowMapDerivativeState(ad_A);
    if (!A.isApprox(ad_A, precision)) {
      std::cout << "A:    " << A.transpose() << std::endl;
      std::cout << "al_A: " << ad_A.transpose() << std::endl;
      success = false;
    }

    matrix_t B, ad_B;
    linearSystem1->getFlowMapDerivativeInput(B);
    linearSystem2->getFlowMapDerivativeInput(ad_B);
    if (!B.isApprox(ad_B, precision)) {
      std::cout << "B:    " << B.transpose() << std::endl;
      std::cout << "al_B: " << ad_B.transpose() << std::endl;
      success = false;
    }

    matrix_t G, ad_G;
    linearSystem1->getJumpMapDerivativeState(G);
    linearSystem2->getJumpMapDerivativeState(ad_G);
    if (!G.isApprox(ad_G, precision)) {
      std::cout << "G:    " << G.transpose() << std::endl;
      std::cout << "al_G: " << ad_G.transpose() << std::endl;
      success = false;
    }
  }  // end of for loop
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, system_dynamics_test) {
  bool success;
  checkSystemDynamics(100, linearSystem_.get(), adLinearSystem_.get(), success, stateDim_, inputDim_);

  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, clone_test) {
  std::unique_ptr<DerivativesBase> adLinearSystemPtr(adLinearSystem_->clone());
  bool success;
  checkSystemDynamics(100, linearSystem_.get(), adLinearSystemPtr.get(), success, stateDim_, inputDim_);

  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, multithread_test) {
  std::unique_ptr<DerivativesBase> linearSystemPtr(linearSystem_->clone());
  std::unique_ptr<DerivativesBase> adLinearSystemPtr(adLinearSystem_->clone());

  bool success = false;
  std::thread thread1(checkSystemDynamics, 10000, linearSystem_.get(), adLinearSystem_.get(), std::ref(success), stateDim_, inputDim_);

  bool successClone = false;
  std::thread thread2(checkSystemDynamics, 10000, linearSystemPtr.get(), adLinearSystemPtr.get(), std::ref(successClone), stateDim_,
                      inputDim_);

  if (thread1.joinable()) {
    thread1.join();
  }
  if (thread2.joinable()) {
    thread2.join();
  }

  ASSERT_TRUE(success && successClone);
}
