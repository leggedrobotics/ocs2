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

#include "ocs2_qp_solver/wrappers/SystemWrapper.h"

#include "testProblemsGeneration.h"

class SystemWrapperTest : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  using SystemDynamics_t = ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>;
  using scalar_t = SystemDynamics_t::scalar_t;
  using state_input_matrix_t = SystemDynamics_t::state_input_matrix_t;
  using input_vector_t = SystemDynamics_t::input_vector_t;
  using state_matrix_t = SystemDynamics_t::state_matrix_t;
  using state_vector_t = SystemDynamics_t::state_vector_t;

  SystemWrapperTest() {
    // Construct random system
    srand(0);
    system = ocs2::qp_solver::getOcs2Dynamics<STATE_DIM, INPUT_DIM>(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    systemWrapper.reset(new ocs2::qp_solver::SystemWrapper(*system));

    // Setpoint
    t = 0.42;
    x = state_vector_t::Random();
    u = input_vector_t::Random();

    // Prepare cost at setpoint after wrapping
    system->setCurrentStateAndControl(t, x, u);
  }

  double t;
  state_vector_t x;
  input_vector_t u;
  std::unique_ptr<ocs2::qp_solver::SystemWrapper> systemWrapper;
  std::unique_ptr<SystemDynamics_t> system;
};

TEST_F(SystemWrapperTest, flowMap) {
  state_vector_t dxdt;
  system->computeFlowMap(t, x, u, dxdt);
  ASSERT_TRUE(dxdt.isApprox(systemWrapper->getFlowMap(t, x, u)));
}

TEST_F(SystemWrapperTest, flowMapAfterCopy) {
  state_vector_t dxdt;
  system->computeFlowMap(t, x, u, dxdt);
  system.reset();
  // Copy and destroy old wrapper
  auto wrapperClone = *systemWrapper;
  systemWrapper.reset();
  ASSERT_TRUE(dxdt.isApprox(wrapperClone.getFlowMap(t, x, u)));
}

TEST_F(SystemWrapperTest, flowMapLinearApproximation) {
  // Define deviation
  double dt = 0.24;
  state_vector_t dx = state_vector_t::Random();
  input_vector_t du = input_vector_t::Random();

  // flowMap at deviation
  state_vector_t dxdt_true;
  system->computeFlowMap(t + dt, x + dx, u + du, dxdt_true);
  system.reset();  // Destroy the cost function after evaluation

  const auto linearApproximation = systemWrapper->getLinearApproximation(t, x, u);
  const auto& dfdx = linearApproximation.dfdx;
  const auto& dfdu = linearApproximation.dfdu;
  const auto& f = linearApproximation.f;
  Eigen::VectorXd dxdt_wrapped_approximation = dfdx * dx + dfdu * du + f;

  ASSERT_TRUE(dxdt_true.isApprox(dxdt_wrapped_approximation));
}