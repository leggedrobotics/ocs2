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
#include <iostream>

#include "ocs2_frank_wolfe/GradientDescent.h"

using namespace ocs2;

class QuadraticCost final : public NLP_Cost {
 public:
  QuadraticCost() = default;
  ~QuadraticCost() = default;

  size_t setCurrentParameter(const vector_t& x) override {
    x_ = x;
    return 0;
  }

  bool getCost(size_t id, scalar_t& f) override {
    f = 0.5 * x_.dot(x_);
    return true;
  }

  void getCostDerivative(size_t id, vector_t& g) override { g = x_; }

  void getCostSecondDerivative(size_t id, matrix_t& H) override { H.setIdentity(x_.size(), x_.size()); }

  void clearCache() override {}

 private:
  vector_t x_;
};

class QuadraticConstraints final : public NLP_Constraints {
 public:
  QuadraticConstraints(vector_t minX, vector_t maxX) {
    size_t numParameters = maxX.size();

    Cm_.resize(2 * numParameters, numParameters);
    Dv_.resize(2 * numParameters);

    Cm_.topRows(numParameters) = -Eigen::MatrixXd::Identity(numParameters, numParameters);
    Cm_.bottomRows(numParameters) = Eigen::MatrixXd::Identity(numParameters, numParameters);

    Dv_.head(numParameters) = maxX;
    Dv_.tail(numParameters) = -minX;
  }

  ~QuadraticConstraints() = default;

  void setCurrentParameter(const vector_t& x) override { x_ = x; }

  void getLinearInequalityConstraint(vector_t& h) override { h = Cm_ * x_ + Dv_; }

  void getLinearInequalityConstraintDerivative(matrix_t& dhdx) override { dhdx = Cm_; }

 private:
  vector_t x_;
  matrix_t Cm_;
  vector_t Dv_;
};

TEST(QuadraticTest, QuadraticTest) {
  NLP_Settings nlpSettings;
  nlpSettings.displayInfo_ = true;
  nlpSettings.maxIterations_ = 500;
  nlpSettings.minRelCost_ = 1e-6;
  nlpSettings.maxLearningRate_ = 1.0;
  nlpSettings.minLearningRate_ = 1e-4;
  nlpSettings.useAscendingLineSearchNLP_ = false;

  GradientDescent nlpSolver(nlpSettings);
  std::unique_ptr<QuadraticCost> costPtr(new QuadraticCost);

  vector_t maxX = Eigen::Vector2d(3.0, 3.0);
  vector_t minX = Eigen::Vector2d(1.0, 1.0);
  std::unique_ptr<QuadraticConstraints> constraintsPtr(new QuadraticConstraints(minX, maxX));

  Eigen::Vector2d initParameters = 0.5 * (maxX + minX) + 0.5 * (maxX - minX).cwiseProduct(Eigen::Vector2d::Random());
  nlpSolver.run(initParameters, 0.1 * Eigen::Vector2d::Ones(), costPtr.get(), constraintsPtr.get());

  double cost;
  nlpSolver.getCost(cost);
  vector_t parameters;
  nlpSolver.getParameters(parameters);

  std::cout << "cost: " << cost << std::endl;
  std::cout << "parameters: " << parameters.transpose() << std::endl;

  const double optimalCost = 1.0;
  const Eigen::Vector2d optimalParameters = Eigen::Vector2d::Ones();

  ASSERT_NEAR(cost, optimalCost, nlpSettings.minRelCost_) << "MESSAGE: Frank_Wolfe failed in the Quadratic test!";
}
