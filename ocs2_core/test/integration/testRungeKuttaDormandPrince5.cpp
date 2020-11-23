/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/integration/Integrator.h>

TEST(RungeKuttaDormandPrince5Test, instantiate) {
  auto ode45 = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);
}

std::unique_ptr<ocs2::OdeBase> getLinearSystem() {
  ocs2::matrix_t A(2, 2);
  A << -2, -1,  // clang-format off
        1,  0;  // clang-format on
  ocs2::matrix_t B(2, 1);
  B << 1, 0;

  auto sys = std::unique_ptr<ocs2::ControlledSystemBase>(new ocs2::LinearSystemDynamics(A, B));

  ocs2::vector_array_t uff(2, ocs2::vector_t::Ones(1));
  ocs2::matrix_array_t k(2, ocs2::matrix_t::Zero(1, 2));
  auto controller = new ocs2::LinearController({0, 10}, uff, k);
  sys->setController(controller);

  return sys;
}

TEST(RungeKuttaDormandPrince5Test, integrateLinearSystem) {
  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory;

  auto sys = getLinearSystem();
  auto integrator = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);

  const ocs2::scalar_t t0 = 0.0;
  const ocs2::scalar_t t1 = 10.0;
  const ocs2::scalar_t dt = 0.05;
  ocs2::vector_t x0 = ocs2::vector_t::Zero(2);
  ocs2::Observer observer(&stateTrajectory, &timeTrajectory);
  integrator->integrateAdaptive(*sys, observer, x0, t0, t1, dt);

  EXPECT_NEAR(timeTrajectory.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);
}

TEST(RungeKuttaDormandPrince5Test, compareWithBoostOdeint) {
  const ocs2::scalar_t t0 = 0.0;
  const ocs2::scalar_t t1 = 10.0;
  const ocs2::scalar_t dt = 0.05;
  const ocs2::vector_t x0 = ocs2::vector_t::Zero(2);

  auto sys = getLinearSystem();

  ocs2::scalar_array_t tTraj;
  ocs2::vector_array_t xTraj;
  ocs2::Observer observer(&xTraj, &tTraj);
  auto integrator = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);
  integrator->integrateAdaptive(*sys, observer, x0, t0, t1, dt);

  ocs2::scalar_array_t tTraj_boost;
  ocs2::vector_array_t xTraj_boost;
  ocs2::Observer observer_boost(&xTraj_boost, &tTraj_boost);
  auto integrator_boost = ocs2::newIntegrator(ocs2::IntegratorType::ODE45);
  integrator_boost->integrateAdaptive(*sys, observer_boost, x0, t0, t1, dt);

  for (size_t i = 0; i < tTraj.size(); i++) {
    // std::cout << "t " << tTraj[i] << "  t_boost " << tTraj_boost[i] << '\n';
    EXPECT_NEAR(tTraj[i], tTraj_boost[i], 1e-6);
    EXPECT_TRUE(xTraj[i].isApprox(xTraj_boost[i], 1e-6));
  }
}

TEST(RungeKuttaDormandPrince5Test, integrateBackwards) {
  // TODO
}
