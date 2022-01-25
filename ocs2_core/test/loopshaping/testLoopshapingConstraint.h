/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingPreComputation.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraint.h"

#include "testLoopshapingConfigurations.h"
#include "testQuadraticConstraint.h"

namespace ocs2 {

class TestFixtureLoopShapingConstraint : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingConstraint(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    // Make system Constraint
    systemConstraint = TestQuadraticStateInputConstraint::createRandom(systemStateDim_, inputDim_);
    systemStateConstraint = TestQuadraticStateConstraint::createRandom(systemStateDim_);

    // Create temporary collection and add a constraint copy
    StateInputConstraintCollection systemConstraintCollection;
    StateConstraintCollection systemStateConstraintCollection;
    systemConstraintCollection.add("", std::unique_ptr<StateInputConstraint>(systemConstraint->clone()));
    systemStateConstraintCollection.add("", std::unique_ptr<StateConstraint>(systemStateConstraint->clone()));

    // Create Loopshaping constraint
    loopshapingConstraint = LoopshapingConstraint::create(systemConstraintCollection, loopshapingDefinition_);
    loopshapingStateConstraint = LoopshapingConstraint::create(systemStateConstraintCollection, loopshapingDefinition_);
  };

  void testStateInputConstraintEvaluation() const {
    // Evaluate system
    vector_t g_system = systemConstraint->getValue(t, x_sys_, u_sys_, PreComputation());

    // Evaluate loopshaping system
    preComp_->request(Request::Constraint, t, x_, u_);
    vector_t g = loopshapingConstraint->getValue(t, x_, u_, *preComp_);

    // The constraint should stay the same
    EXPECT_LE((g_system - g).array().abs().maxCoeff(), tol);
  }

  void testStateInputConstraintLinearApproximation() const {
    // Extract approximation
    preComp_->request(Request::Constraint + Request::Approximation, t, x_, u_);
    const auto g_linear = loopshapingConstraint->getLinearApproximation(t, x_, u_, *preComp_);
    const auto g_quadratic = loopshapingConstraint->getQuadraticApproximation(t, x_, u_, *preComp_);

    EXPECT_TRUE(g_linear.f.isApprox(g_quadratic.f));
    EXPECT_TRUE(g_linear.dfdx.isApprox(g_quadratic.dfdx));
    EXPECT_TRUE(g_linear.dfdu.isApprox(g_quadratic.dfdu));
  }

  void testStateInputConstraintQuadraticApproximation() const {
    // Extract approximation
    preComp_->request(Request::Constraint + Request::Approximation, t, x_, u_);
    const auto h = loopshapingConstraint->getQuadraticApproximation(t, x_, u_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->request(Request::Constraint, t, x_ + x_disturbance_, u_ + u_disturbance_);
    vector_t h_disturbance = loopshapingConstraint->getValue(t, x_ + x_disturbance_, u_ + u_disturbance_, *preComp_);

    // Evaluate approximation
    for (size_t i = 0; i < h.f.rows(); i++) {
      scalar_t h_approximation = h.f(i) + h.dfdx.row(i) * x_disturbance_ + h.dfdu.row(i) * u_disturbance_ +
                                 0.5 * x_disturbance_.transpose() * h.dfdxx[i] * x_disturbance_ +
                                 0.5 * u_disturbance_.transpose() * h.dfduu[i] * u_disturbance_ +
                                 u_disturbance_.transpose() * h.dfdux[i] * x_disturbance_;
      EXPECT_LE(std::abs(h_disturbance[i] - h_approximation), tol);
    }
  }

  void testStateOnlyConstraintEvaluation() const {
    // Evaluate system
    vector_t g_system = systemStateConstraint->getValue(t, x_sys_, PreComputation());

    // Evaluate loopshaping system
    preComp_->requestFinal(Request::Constraint, t, x_);
    vector_t g = loopshapingStateConstraint->getValue(t, x_, *preComp_);

    // System part of the constraints should stay the same
    EXPECT_LE((g_system - g).array().abs().maxCoeff(), tol);
  }

  void testStateOnlyConstraintLinearApproximation() const {
    // Extract approximation
    preComp_->requestFinal(Request::Constraint + Request::Approximation, t, x_);
    const auto g_linear = loopshapingStateConstraint->getLinearApproximation(t, x_, *preComp_);
    const auto g_quadratic = loopshapingStateConstraint->getQuadraticApproximation(t, x_, *preComp_);

    EXPECT_TRUE(g_linear.f.isApprox(g_quadratic.f));
    EXPECT_TRUE(g_linear.dfdx.isApprox(g_quadratic.dfdx));
  }

  void testStateOnlyConstraintQuadraticApproximation() const {
    // Extract approximation
    preComp_->requestFinal(Request::Constraint + Request::Approximation, t, x_);
    const auto h = loopshapingStateConstraint->getQuadraticApproximation(t, x_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->requestFinal(Request::Constraint, t, x_ + x_disturbance_);
    vector_t h_disturbance = loopshapingStateConstraint->getValue(t, x_ + x_disturbance_, *preComp_);

    // Evaluate approximation
    for (size_t i = 0; i < h.f.rows(); i++) {
      scalar_t h_approximation = h.f(i) + h.dfdx.row(i) * x_disturbance_ + 0.5 * x_disturbance_.transpose() * h.dfdxx[i] * x_disturbance_;
      EXPECT_LE(std::abs(h_disturbance[i] - h_approximation), tol);
    }
  }

 private:
  std::unique_ptr<TestQuadraticStateInputConstraint> systemConstraint;
  std::unique_ptr<TestQuadraticStateConstraint> systemStateConstraint;
  std::unique_ptr<StateInputConstraintCollection> loopshapingConstraint;
  std::unique_ptr<StateConstraintCollection> loopshapingStateConstraint;
};

}  // namespace ocs2