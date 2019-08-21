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

class testCppADCG_dynamicsFixture : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const size_t state_dim_ = 4;
  static const size_t input_dim_ = 2;

  using base_dynamics_t = ocs2::DerivativesBase<state_dim_, input_dim_>;
  using system_dynamics_t = ocs2::LinearSystemDynamics<state_dim_, input_dim_>;
  using ad_system_dynamics_t = ocs2::LinearSystemDynamicsAD<state_dim_, input_dim_>;

  using scalar_t = typename system_dynamics_t::scalar_t;
  using state_vector_t = typename system_dynamics_t::state_vector_t;
  using state_matrix_t = typename system_dynamics_t::state_matrix_t;
  using input_vector_t = typename system_dynamics_t::input_vector_t;
  using state_input_matrix_t = typename system_dynamics_t::state_input_matrix_t;

  testCppADCG_dynamicsFixture() { create(); };

  void create() {
    // Define dynamics parameters
    state_matrix_t A = state_matrix_t::Random();
    state_input_matrix_t B = state_input_matrix_t::Random();
    state_matrix_t G = state_matrix_t::Random();

    linearSystem_.reset(new system_dynamics_t(A, B, G));

    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
    adLinearSystem_.reset(new ad_system_dynamics_t(A, B, G));

    adLinearSystem_->initialize("testCppADCG_dynamics", libraryFolder, true, true);
  }

  std::unique_ptr<system_dynamics_t> linearSystem_;
  std::unique_ptr<ad_system_dynamics_t> adLinearSystem_;
};

/******************************************************************************/
/******************************************************************************/
/***************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void checkSystemDynamics(const size_t numTests, ocs2::DerivativesBase<STATE_DIM, INPUT_DIM>* const linearSystem1,
                         ocs2::DerivativesBase<STATE_DIM, INPUT_DIM>* const linearSystem2, bool& success) {
  using system_dynamics_t = ocs2::DerivativesBase<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename system_dynamics_t::scalar_t;
  using state_vector_t = typename system_dynamics_t::state_vector_t;
  using state_matrix_t = typename system_dynamics_t::state_matrix_t;
  using input_vector_t = typename system_dynamics_t::input_vector_t;
  using state_input_matrix_t = typename system_dynamics_t::state_input_matrix_t;

  success = true;

  const scalar_t precision = 1e-9;

  state_vector_t x;
  input_vector_t u;

  for (size_t it = 0; it < numTests; it++) {
    x.setRandom();
    u.setRandom();

    linearSystem1->setCurrentStateAndControl(0.0, x, u);
    linearSystem2->setCurrentStateAndControl(0.0, x, u);

    state_matrix_t A, ad_A;
    linearSystem1->getFlowMapDerivativeState(A);
    linearSystem2->getFlowMapDerivativeState(ad_A);
    if (!A.isApprox(ad_A, precision)) {
      std::cout << "A:    " << A.transpose() << std::endl;
      std::cout << "al_A: " << ad_A.transpose() << std::endl;
      success = false;
    }

    state_input_matrix_t B, ad_B;
    linearSystem1->getFlowMapDerivativeInput(B);
    linearSystem2->getFlowMapDerivativeInput(ad_B);
    if (!B.isApprox(ad_B, precision)) {
      std::cout << "B:    " << B.transpose() << std::endl;
      std::cout << "al_B: " << ad_B.transpose() << std::endl;
      success = false;
    }

    state_matrix_t G, ad_G;
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
  checkSystemDynamics(100, linearSystem_.get(), adLinearSystem_.get(), success);

  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, clone_test) {
  std::unique_ptr<base_dynamics_t> adLinearSystemPtr(adLinearSystem_->clone());

  bool success;
  checkSystemDynamics(100, linearSystem_.get(), adLinearSystemPtr.get(), success);

  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, multithread_test) {
  std::unique_ptr<base_dynamics_t> linearSystemPtr(linearSystem_->clone());
  std::unique_ptr<base_dynamics_t> adLinearSystemPtr(adLinearSystem_->clone());

  bool success = false;
  std::thread thread1(checkSystemDynamics<state_dim_, input_dim_>, 10000, linearSystem_.get(), adLinearSystem_.get(), std::ref(success));

  bool successClone = false;
  std::thread thread2(checkSystemDynamics<state_dim_, input_dim_>, 10000, linearSystemPtr.get(), adLinearSystemPtr.get(),
                      std::ref(successClone));

  if (thread1.joinable()) thread1.join();
  if (thread2.joinable()) thread2.join();

  ASSERT_TRUE(success && successClone);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
