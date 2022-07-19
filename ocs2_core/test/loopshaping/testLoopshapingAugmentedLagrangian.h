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

#include "ocs2_core/augmented_lagrangian/AugmentedLagrangian.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingPreComputation.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"
#include "ocs2_core/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangian.h"
#include "ocs2_core/penalties/Penalties.h"

#include "testLoopshapingConfigurations.h"
#include "testQuadraticConstraint.h"

namespace ocs2 {

class TestFixtureLoopShapingAugmentedLagrangian : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingAugmentedLagrangian(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    // Make system Constraint
    auto systemConstraint = TestQuadraticStateInputConstraint::createRandom(systemStateDim_, inputDim_);
    auto systemStateConstraint = TestQuadraticStateConstraint::createRandom(systemStateDim_);

    // Make Lagrangian: we use zero scale factor in order to make the quadratic approximation exact
    systemAugmentedLagrangian = create(std::move(systemConstraint), augmented::QuadraticPenalty::create({0.0, 1.0}));
    systemStateAugmentedLagrangian = create(std::move(systemStateConstraint), augmented::QuadraticPenalty::create({0.0, 1.0}));

    // Create temporary collection and add a constraint copy
    StateInputAugmentedLagrangianCollection systemAugmentedLagrangianCollection;
    StateAugmentedLagrangianCollection systemStateAugmentedLagrangianCollection;

    systemAugmentedLagrangianCollection.add("", std::unique_ptr<StateInputAugmentedLagrangian>(systemAugmentedLagrangian->clone()));
    systemStateAugmentedLagrangianCollection.add("", std::unique_ptr<StateAugmentedLagrangian>(systemStateAugmentedLagrangian->clone()));

    // Create Loopshaping constraint
    loopshapingAugmentedLagrangian = LoopshapingAugmentedLagrangian::create(systemAugmentedLagrangianCollection, loopshapingDefinition_);
    loopshapingStateAugmentedLagrangian =
        LoopshapingAugmentedLagrangian::create(systemStateAugmentedLagrangianCollection, loopshapingDefinition_);
  };

  void testStateInputEvaluation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->getNumConstraints(t);
    const auto termsMultiplier = toMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Evaluate system
    const auto metrics_system = systemAugmentedLagrangian->getValue(t, x_sys_, u_sys_, termsMultiplier[0], PreComputation());

    // Evaluate loopshaping system
    preComp_->request(Request::Constraint, t, x_, u_);
    const auto metrics = loopshapingAugmentedLagrangian->getValue(t, x_, u_, termsMultiplier, *preComp_)[0];

    // The constraint and penalty should stay the same
    EXPECT_NEAR(metrics_system.penalty, metrics.penalty, tol);
    EXPECT_TRUE(metrics_system.constraint.isApprox(metrics.constraint, tol));
  }

  void testStateInputApproximation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->getNumConstraints(t);
    const auto termsMultiplier = toMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Extract Quadratic approximation
    preComp_->request(Request::Constraint + Request::Approximation, t, x_, u_);
    const auto L = loopshapingAugmentedLagrangian->getQuadraticApproximation(t, x_, u_, termsMultiplier, *preComp_);

    // Reevaluate at disturbed state
    preComp_->request(Request::Constraint, t, x_ + x_disturbance_, u_ + u_disturbance_);
    const scalar_t L_disturbance =
        sumPenalties(loopshapingAugmentedLagrangian->getValue(t, x_ + x_disturbance_, u_ + u_disturbance_, termsMultiplier, *preComp_));

    // Evaluate approximation
    const scalar_t L_quad_approximation = L.f + L.dfdx.dot(x_disturbance_) + L.dfdu.dot(u_disturbance_) +
                                          0.5 * x_disturbance_.dot(L.dfdxx * x_disturbance_) +
                                          0.5 * u_disturbance_.dot(L.dfduu * u_disturbance_) + u_disturbance_.dot(L.dfdux * x_disturbance_);

    // Difference between new evaluation and approximation should be less than tol
    EXPECT_NEAR(L_disturbance, L_quad_approximation, tol);
  }

  void testStateEvaluation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->getNumConstraints(t);
    const auto termsMultiplier = toMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Evaluate system
    const auto metrics_system = systemStateAugmentedLagrangian->getValue(t, x_sys_, termsMultiplier[0], PreComputation());

    // Evaluate loopshaping system
    preComp_->requestFinal(Request::Constraint, t, x_);
    const auto metrics = loopshapingStateAugmentedLagrangian->getValue(t, x_, termsMultiplier, *preComp_)[0];

    // The constraint and penalty should stay the same
    EXPECT_NEAR(metrics_system.penalty, metrics.penalty, tol);
    EXPECT_TRUE(metrics_system.constraint.isApprox(metrics.constraint, tol));
  }

  void testStateApproximation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->getNumConstraints(t);
    const auto termsMultiplier = toMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Extract Quadratic approximation
    preComp_->requestFinal(Request::Constraint + Request::Approximation, t, x_);
    const auto L = loopshapingStateAugmentedLagrangian->getQuadraticApproximation(t, x_, termsMultiplier, *preComp_);

    // Reevaluate at disturbed state
    preComp_->requestFinal(Request::Constraint, t, x_ + x_disturbance_);
    const scalar_t L_disturbance =
        sumPenalties(loopshapingStateAugmentedLagrangian->getValue(t, x_ + x_disturbance_, termsMultiplier, *preComp_));

    // Evaluate approximation
    const scalar_t L_quad_approximation = L.f + L.dfdx.dot(x_disturbance_) + 0.5 * x_disturbance_.dot(L.dfdxx * x_disturbance_);

    // Difference between new evaluation and approximation should be less than tol
    EXPECT_NEAR(L_disturbance, L_quad_approximation, tol);
  }

 private:
  std::unique_ptr<StateInputAugmentedLagrangian> systemAugmentedLagrangian;
  std::unique_ptr<StateAugmentedLagrangian> systemStateAugmentedLagrangian;

  std::unique_ptr<StateInputAugmentedLagrangianCollection> loopshapingAugmentedLagrangian;
  std::unique_ptr<StateAugmentedLagrangianCollection> loopshapingStateAugmentedLagrangian;
};

}  // namespace ocs2
