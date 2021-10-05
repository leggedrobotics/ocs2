

#pragma once

#include "testLoopshapingConfigurations.h"

#include "ocs2_core/dynamics/LinearSystemDynamics.h"
#include "ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h"

namespace ocs2 {

class TestFixtureLoopShapingDynamics : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingDynamics(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    // Create system dynamics
    matrix_t A, B, G;
    A.setRandom(systemStateDim_, systemStateDim_);
    B.setRandom(systemStateDim_, inputDim_);
    G.setRandom(systemStateDim_, systemStateDim_);
    testSystem.reset(new LinearSystemDynamics(A, B, G));

    // Create Loopshaping Dynamics
    testLoopshapingDynamics = LoopshapingDynamics::create(*testSystem, loopshapingDefinition_);
  };

  void evaluateDynamics() const {
    // Evaluate system
    preComp_sys_->request(Request::Dynamics, t, x_, u_);
    vector_t dx_sys = testSystem->computeFlowMap(t, x_sys_, u_sys_, *preComp_sys_);

    // Evaluate loopshaping system
    preComp_->request(Request::Dynamics, t, x_, u_);
    vector_t dx = testLoopshapingDynamics->computeFlowMap(t, x_, u_, *preComp_);

    // System part of the flowmap should stay the same
    ASSERT_TRUE(dx_sys.isApprox(dx.head(dx_sys.rows())));
  }

  void evaluateDynamicsApproximation() const {
    // Extract linearization
    preComp_->request(Request::Dynamics + Request::Approximation, t, x_, u_);
    const auto linearization = testLoopshapingDynamics->linearApproximation(t, x_, u_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->request(Request::Dynamics, t, x_ + x_disturbance_, u_ + u_disturbance_);
    vector_t dx_disturbance = testLoopshapingDynamics->computeFlowMap(t, x_ + x_disturbance_,
                                                                            u_ + u_disturbance_, *preComp_);

    // Evaluate approximation
    vector_t dx_approximation = linearization.f + linearization.dfdx * x_disturbance_ + linearization.dfdu * u_disturbance_;

    // Difference between new evaluation and linearization should be less than tol
    ASSERT_LE((dx_disturbance - dx_approximation).array().abs().maxCoeff(), tol);
  }

  void evaluateJumpMap() const {
    // Evaluate jump map
    preComp_sys_->requestPreJump(Request::Dynamics, t, x_sys_);
    const vector_t jumpMap_sys = testSystem->computeJumpMap(t, x_sys_, *preComp_sys_);

    preComp_->requestPreJump(Request::Dynamics, t, x_);
    const vector_t jumpMap = testLoopshapingDynamics->computeJumpMap(t, x_, *preComp_);

    EXPECT_TRUE(jumpMap.head(x_sys_.rows()).isApprox(jumpMap_sys));
  }

  void evaluateJumpMapApproximation() {
    // Evaluate linearization
    preComp_sys_->requestPreJump(Request::Dynamics + Request::Approximation, t, x_sys_);
    const auto jumpMap_sys = testSystem->jumpMapLinearApproximation(t, x_sys_, *preComp_sys_);

    preComp_->requestPreJump(Request::Dynamics + Request::Approximation, t, x_);
    const auto jumpMap = testLoopshapingDynamics->jumpMapLinearApproximation(t, x_, *preComp_);

    EXPECT_TRUE(jumpMap.f.head(x_sys_.rows()).isApprox(jumpMap_sys.f));
    EXPECT_TRUE(jumpMap.dfdx.topLeftCorner(x_sys_.rows(), x_sys_.rows()).isApprox(jumpMap_sys.dfdx));
    EXPECT_TRUE(jumpMap.dfdu.size() == 0);
    EXPECT_TRUE(jumpMap_sys.dfdu.size() == 0);
  }

 private:
  std::unique_ptr<LinearSystemDynamics> testSystem;
  std::unique_ptr<LoopshapingDynamics> testLoopshapingDynamics;
};

};  // namespace ocs2
