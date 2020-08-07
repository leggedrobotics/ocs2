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

#include <functional>
#include <iostream>

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include "../cost/CheckCostFunction.h"
#include "QuadraticCostFunctionAD.h"

using namespace ocs2;

class testCppADCG_costFixture : public ::testing::Test {
 public:
  const size_t stateDim_ = 4;
  const size_t inputDim_ = 2;

  testCppADCG_costFixture() { create(); };

  void create() {
    // Define cost parameters
    matrix_t Q = 5.0 * matrix_t::Random(stateDim_, stateDim_);
    matrix_t R = 3.0 * matrix_t::Random(inputDim_, inputDim_);
    matrix_t P = 2.0 * matrix_t::Random(inputDim_, stateDim_);
    vector_t xNominal = vector_t::Random(stateDim_);
    vector_t uNominal = vector_t::Random(inputDim_);
    matrix_t QFinal = 4.0 * matrix_t::Random(stateDim_, stateDim_);
    Q = (Q + Q.transpose()).eval();
    R = (R + R.transpose()).eval();
    QFinal = (QFinal + QFinal.transpose()).eval();
    costDesiredTrajectories_ = CostDesiredTrajectories({0.0}, {xNominal}, {uNominal});

    quadraticCost_.reset(new QuadraticCostFunction(Q, R, QFinal, P));
    quadraticCost_->setCostDesiredTrajectoriesPtr(&costDesiredTrajectories_);

    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
    adQuadraticCost_.reset(new QuadraticCostFunctionAD(Q, R, xNominal, uNominal, QFinal, xNominal, P));

    adQuadraticCost_->initialize("testCppADCG_cost", libraryFolder, true, true);
  }

  std::unique_ptr<QuadraticCostFunction> quadraticCost_;
  std::unique_ptr<QuadraticCostFunctionAD> adQuadraticCost_;
  CostDesiredTrajectories costDesiredTrajectories_;
};

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_costFixture, quadratic_cost_test) {
  bool success;
  checkCostFunction(100, quadraticCost_.get(), adQuadraticCost_.get(), success, stateDim_, inputDim_);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_costFixture, clone__test) {
  std::unique_ptr<CostFunctionBase> ad_quadraticCostPtr(adQuadraticCost_->clone());
  bool success;
  checkCostFunction(100, quadraticCost_.get(), ad_quadraticCostPtr.get(), success, stateDim_, inputDim_);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_costFixture, multithread_test) {
  std::unique_ptr<CostFunctionBase> quadraticCostPtr(quadraticCost_->clone());
  std::unique_ptr<CostFunctionBase> adQuadraticCostPtr(adQuadraticCost_->clone());

  bool success = false;
  std::thread thread1(checkCostFunction, 10000, quadraticCost_.get(), adQuadraticCost_.get(), std::ref(success), stateDim_, inputDim_);

  bool successClone = false;
  std::thread thread2(checkCostFunction, 10000, quadraticCostPtr.get(), adQuadraticCostPtr.get(), std::ref(successClone), stateDim_,
                      inputDim_);

  if (thread1.joinable()) {
    thread1.join();
  }
  if (thread2.joinable()) {
    thread2.join();
  }

  ASSERT_TRUE(success && successClone);
}
