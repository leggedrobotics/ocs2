

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