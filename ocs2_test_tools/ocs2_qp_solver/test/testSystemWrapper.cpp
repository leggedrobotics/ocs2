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

//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include "ocs2_qp_solver/test/testProblemsGeneration.h"
#include "ocs2_qp_solver/wrappers/SystemWrapper.h"

class SystemWrapperTest : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;

  SystemWrapperTest() {
    // Construct random system
    srand(0);
    system = ocs2::qp_solver::getOcs2Dynamics(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    systemWrapper.reset(new ocs2::qp_solver::SystemWrapper(*system));

    // Setpoint
    t = 0.42;
    x = ocs2::vector_t::Random(STATE_DIM);
    u = ocs2::vector_t::Random(INPUT_DIM);

    // Prepare cost at setpoint after wrapping
    system->setCurrentStateAndControl(t, x, u);
  }

  ocs2::scalar_t t;
  ocs2::vector_t x;
  ocs2::vector_t u;
  std::unique_ptr<ocs2::qp_solver::SystemWrapper> systemWrapper;
  std::unique_ptr<ocs2::SystemDynamicsBase> system;
};

TEST_F(SystemWrapperTest, flowMap) {
  ocs2::vector_t dxdt = system->computeFlowMap(t, x, u);
  ASSERT_TRUE(dxdt.isApprox(systemWrapper->getFlowMap(t, x, u)));
}

TEST_F(SystemWrapperTest, flowMapAfterCopy) {
  ocs2::vector_t dxdt = system->computeFlowMap(t, x, u);
  system.reset();
  // Copy and destroy old wrapper
  auto wrapperClone = *systemWrapper;
  systemWrapper.reset();
  ASSERT_TRUE(dxdt.isApprox(wrapperClone.getFlowMap(t, x, u)));
}

TEST_F(SystemWrapperTest, flowMapLinearApproximation) {
  // Define deviation
  ocs2::scalar_t dt = 0.24;
  ocs2::vector_t dx = ocs2::vector_t::Random(STATE_DIM);
  ocs2::vector_t du = ocs2::vector_t::Random(INPUT_DIM);

  // flowMap at deviation
  ocs2::vector_t dxdt_true = system->computeFlowMap(t + dt, x + dx, u + du);
  system.reset();  // Destroy the cost function after evaluation

  const auto linearApproximation = systemWrapper->getLinearApproximation(t, x, u);
  const auto& dfdx = linearApproximation.dfdx;
  const auto& dfdu = linearApproximation.dfdu;
  const auto& f = linearApproximation.f;
  ocs2::vector_t dxdt_wrapped_approximation = dfdx * dx + dfdu * du + f;

  ASSERT_TRUE(dxdt_true.isApprox(dxdt_wrapped_approximation));
}
