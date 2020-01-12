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

class testCostfunctionLinearCombinationFixture : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const size_t state_dim_ = 4;
  static const size_t input_dim_ = 2;

  using base_cost_t = ocs2::CostFunctionBase<state_dim_, input_dim_>;
  using quadratic_cost_t = ocs2::QuadraticCostFunction<state_dim_, input_dim_>;
  using linear_combination_cost_t = ocs2::CostFunctionLinearCombination<state_dim_, input_dim_>;
  using WeightedCost = linear_combination_cost_t::WeightedCost;

  using scalar_t = base_cost_t::scalar_t;
  using state_vector_t = base_cost_t::state_vector_t;
  using state_matrix_t = base_cost_t::state_matrix_t;
  using input_vector_t = base_cost_t::input_vector_t;
  using input_matrix_t = base_cost_t::input_matrix_t;
  using input_state_matrix_t = base_cost_t::input_state_matrix_t;

  testCostfunctionLinearCombinationFixture() { create(); };

  void create() {
    // Define cost parameters
    state_matrix_t Q = 5.0 * state_matrix_t::Random();
    input_matrix_t R = 3.0 * input_matrix_t::Random();
    input_state_matrix_t P = 2.0 * input_state_matrix_t::Random();
    state_vector_t xNominal = state_vector_t::Random();
    input_vector_t uNominal = input_vector_t::Random();
    state_matrix_t QFinal = 4.0 * state_matrix_t::Random();
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

      weightedQuadraticCost.second.reset(new quadratic_cost_t(Q, R, xNominal, uNominal, QFinal, xNominal, P));

      weightedCostVector.push_back(weightedQuadraticCost);
    }

    combinedCost_.reset(new linear_combination_cost_t(weightedCostVector));
    quadraticCost_.reset(new quadratic_cost_t(accumulatedWeights * Q, accumulatedWeights * R, xNominal, uNominal,
                                              accumulatedWeights * QFinal, xNominal, accumulatedWeights * P));
  }

  std::unique_ptr<linear_combination_cost_t> combinedCost_;
  std::unique_ptr<quadratic_cost_t> quadraticCost_;
};

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCostfunctionLinearCombinationFixture, quadratic_cost_test) {
  bool success;
  checkCostFunction(100, quadraticCost_.get(), combinedCost_.get(), success);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCostfunctionLinearCombinationFixture, clone_test) {
  base_cost_t::Ptr combinedCostPtr(combinedCost_->clone());

  bool success;
  checkCostFunction(100, quadraticCost_.get(), combinedCostPtr.get(), success);
  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCostfunctionLinearCombinationFixture, multithread_test) {
  std::unique_ptr<base_cost_t> quadraticCostPtr(quadraticCost_->clone());
  std::unique_ptr<base_cost_t> combinedCostPtr(combinedCost_->clone());

  bool success = false;
  std::thread thread1(checkCostFunction<state_dim_, input_dim_>, 10000, quadraticCost_.get(), combinedCost_.get(), std::ref(success));

  bool successClone = false;
  std::thread thread2(checkCostFunction<state_dim_, input_dim_>, 10000, quadraticCostPtr.get(), combinedCostPtr.get(),
                      std::ref(successClone));

  if (thread1.joinable()) {
    thread1.join();
  }
  if (thread2.joinable()) {
    thread2.join();
  }

  ASSERT_TRUE(success && successClone);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
