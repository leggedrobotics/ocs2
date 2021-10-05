

#pragma once

#include <ocs2_core/test/testTools.h>
#include "ocs2_core/cost/QuadraticStateCost.h"
#include "ocs2_core/cost/QuadraticStateInputCost.h"
#include "ocs2_core/loopshaping/cost/LoopshapingCost.h"
#include "ocs2_core/loopshaping/soft_constraint/LoopshapingSoftConstraint.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

class TestFixtureLoopShapingSoftConstraint : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingSoftConstraint(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    matrix_t Q, Q_final, R, P;
    Q.setRandom(systemStateDim_, systemStateDim_);
    Q_final.setRandom(systemStateDim_, systemStateDim_);
    R.setRandom(inputDim_, inputDim_);
    P.setRandom(inputDim_, systemStateDim_);

    // Make symmetric
    Q_final = (0.5 * Q_final.transpose() + 0.5 * Q_final).eval();
    Q = (0.5 * Q.transpose() + 0.5 * Q).eval();
    R = (0.5 * R.transpose() + 0.5 * R).eval();
    systemCost.reset(new QuadraticStateInputCost(Q, R, P));
    systemStateCost.reset(new QuadraticStateCost(Q_final));

    targetTrajectories_ = TargetTrajectories({0.0}, {x_sys_}, {u_sys_});

    StateInputCostCollection systemCostCollection;
    StateCostCollection systemStateCostCollection;
    systemCostCollection.add("", std::unique_ptr<StateInputCost>(systemCost->clone()));
    systemStateCostCollection.add("", std::unique_ptr<StateCost>(systemStateCost->clone()));

    // Create Loopshaping cost collection wrappers
    loopshapingSoftConstraint = LoopshapingSoftConstraint::create(systemCostCollection, loopshapingDefinition_);
    loopshapingStateSoftConstraint = LoopshapingSoftConstraint::create(systemStateCostCollection, loopshapingDefinition_);
  };

  void testStateInputApproximation() const {
    // Extract Quadratic approximation
    preComp_->request(Request::SoftConstraint + Request::Approximation, t, x_, u_);
    const auto L = loopshapingSoftConstraint->getQuadraticApproximation(t, x_, u_, targetTrajectories_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->request(Request::SoftConstraint, t, x_ + x_disturbance_, u_ + u_disturbance_);
    scalar_t L_disturbance = loopshapingSoftConstraint->getValue(t, x_ + x_disturbance_, u_ + u_disturbance_, targetTrajectories_, *preComp_);

    // Evaluate approximation
    scalar_t L_quad_approximation = L.f + L.dfdx.transpose() * x_disturbance_ + L.dfdu.transpose() * u_disturbance_ +
        0.5 * x_disturbance_.transpose() * L.dfdxx * x_disturbance_ +
        0.5 * u_disturbance_.transpose() * L.dfduu * u_disturbance_ +
        u_disturbance_.transpose() * L.dfdux * x_disturbance_;

    // Difference between new evaluation and approximation should be less than tol
    ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), tol);
  }

  void testStateApproximation() const {
    preComp_->requestFinal(Request::SoftConstraint + Request::Approximation, t, x_);

    // Extract Quadratic approximation
    const auto L = loopshapingStateSoftConstraint->getQuadraticApproximation(t, x_, targetTrajectories_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->requestFinal(Request::SoftConstraint, t, x_ + x_disturbance_);
    scalar_t L_disturbance = loopshapingStateSoftConstraint->getValue(t, x_ + x_disturbance_, targetTrajectories_, *preComp_);

    // Evaluate approximation
    scalar_t L_quad_approximation = L.f + L.dfdx.transpose() * x_disturbance_ + 0.5 * x_disturbance_.transpose() * L.dfdxx * x_disturbance_;

    // Difference between new evaluation and approximation should be less than tol
    ASSERT_LE(std::abs(L_disturbance - L_quad_approximation), tol);
  }

 private:
  std::unique_ptr<StateInputCost> systemCost;
  std::unique_ptr<StateCost> systemStateCost;
  std::unique_ptr<StateInputCostCollection> loopshapingSoftConstraint;
  std::unique_ptr<StateCostCollection> loopshapingStateSoftConstraint;
  TargetTrajectories targetTrajectories_;
};

};  // namespace ocs2
