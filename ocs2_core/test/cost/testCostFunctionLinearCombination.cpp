/******************************************************************************
Copyright (c) 2019, Johannes Pankert. All rights reserved.

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
#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <functional>
#include <iostream>
#include <thread>
#include "CheckCostFunction.h"

#include "ocs2_core/cost/CostFunctionLinearCombination.h"
#include "ocs2_core/cost/QuadraticCostFunction.h"

using namespace ocs2;

class testCostfunctionLinearCombinationFixture : public ::testing::Test {
 public:
  static const size_t state_dim_ = 4;
  static const size_t input_dim_ = 2;

  using WeightedCost = CostFunctionLinearCombination::WeightedCost;

  testCostfunctionLinearCombinationFixture() { create(); };

  void create() {
    // Define cost parameters
    matrix_t Q = 5.0 * matrix_t::Random(state_dim_, state_dim_);
    matrix_t R = 3.0 * matrix_t::Random(input_dim_, state_dim_);
    matrix_t P = 2.0 * matrix_t::Random(input_dim_, state_dim_);
    vector_t xNominal = vector_t::Random(state_dim_);
    vector_t uNominal = vector_t::Random(input_dim_);
    matrix_t QFinal = 4.0 * matrix_t::Random(state_dim_, state_dim_);
    Q = (Q + Q.transpose()).eval();
    R = (R + R.transpose()).eval();
    QFinal = (QFinal + QFinal.transpose()).eval();

    double accumulatedWeights = 0;
    std::vector<WeightedCost> weightedCostVector;

    const int nMax = 10;
    int n = rand() % nMax + 1;

    for (int i = 0; i < n; i++) {
      WeightedCost weightedQuadraticCost;

      double weight = rand() / (RAND_MAX + 1.);
      accumulatedWeights += weight;
      weightedQuadraticCost.first = weight;

      weightedQuadraticCost.second.reset(new QuadraticCostFunction(Q, R, xNominal, uNominal, QFinal, xNominal, P));

      weightedCostVector.push_back(weightedQuadraticCost);
    }

    combinedCost_.reset(new CostFunctionLinearCombination(weightedCostVector));
    quadraticCost_.reset(new QuadraticCostFunction(accumulatedWeights * Q, accumulatedWeights * R, xNominal, uNominal,
                                                   accumulatedWeights * QFinal, xNominal, accumulatedWeights * P));
  }

  std::unique_ptr<CostFunctionLinearCombination> combinedCost_;
  std::unique_ptr<QuadraticCostFunction> quadraticCost_;
};

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCostfunctionLinearCombinationFixture, quadratic_cost_test) {
  bool success;
  checkCostFunction(100, quadraticCost_.get(), combinedCost_.get(), success, state_dim_, input_dim_);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCostfunctionLinearCombinationFixture, clone_test) {
  std::unique_ptr<CostFunctionBase> combinedCostPtr(combinedCost_->clone());

  bool success;
  checkCostFunction(100, quadraticCost_.get(), combinedCostPtr.get(), success, state_dim_, input_dim_);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCostfunctionLinearCombinationFixture, multithread_test) {
  std::unique_ptr<CostFunctionBase> quadraticCostPtr(quadraticCost_->clone());
  std::unique_ptr<CostFunctionBase> combinedCostPtr(combinedCost_->clone());

  bool success = false;
  std::thread thread1(checkCostFunction, 10000, quadraticCost_.get(), combinedCost_.get(), std::ref(success), state_dim_, input_dim_);

  bool successClone = false;
  std::thread thread2(checkCostFunction, 10000, quadraticCostPtr.get(), combinedCostPtr.get(), std::ref(successClone), state_dim_,
                      input_dim_);

  if (thread1.joinable()) {
    thread1.join();
  }
  if (thread2.joinable()) {
    thread2.join();
  }

  ASSERT_TRUE(success && successClone);
}
