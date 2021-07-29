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

class LinearSystem final : public ocs2::OdeBase {
 public:
  ~LinearSystem() override = default;
  ocs2::vector_t computeFlowMap(ocs2::scalar_t t, const ocs2::vector_t& x) override {
    const ocs2::matrix_t A = (ocs2::matrix_t(2, 2) << -2, -1,  // clang-format off
                                                       1,  0).finished();  // clang-format on
    const ocs2::vector_t B = (ocs2::vector_t(2) << 1, 0).finished();
    const ocs2::vector_t u = ocs2::vector_t::Ones(1);
    return A * x + B * u;
  }
};

TEST(RungeKuttaDormandPrince5Test, integrateLinearSystem) {
  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory;

  LinearSystem sys;
  auto integrator = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);

  const ocs2::scalar_t t0 = 0.0;
  const ocs2::scalar_t t1 = 10.0;
  const ocs2::scalar_t dt = 0.01;
  ocs2::vector_t x0 = ocs2::vector_t::Zero(2);
  ocs2::Observer observer(&stateTrajectory, &timeTrajectory);
  integrator->integrateAdaptive(sys, observer, x0, t0, t1, dt);

  EXPECT_NEAR(timeTrajectory.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x0));
  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);
}

TEST(RungeKuttaDormandPrince5Test, IntegrateAdaptiveCompareWithBoost) {
  const ocs2::scalar_t t0 = 0.0;
  const ocs2::scalar_t t1 = 10.0;
  const ocs2::scalar_t dt = 0.05;
  const ocs2::vector_t x0 = ocs2::vector_t::Zero(2);

  LinearSystem sys;

  ocs2::scalar_array_t tTraj;
  ocs2::vector_array_t xTraj;
  ocs2::Observer observer(&xTraj, &tTraj);
  auto integrator = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);
  integrator->integrateAdaptive(sys, observer, x0, t0, t1, dt);

  ocs2::scalar_array_t tTraj_boost;
  ocs2::vector_array_t xTraj_boost;
  ocs2::Observer observer_boost(&xTraj_boost, &tTraj_boost);
  auto integrator_boost = ocs2::newIntegrator(ocs2::IntegratorType::ODE45);
  integrator_boost->integrateAdaptive(sys, observer_boost, x0, t0, t1, dt);

  for (size_t i = 0; i < tTraj.size(); i++) {
    EXPECT_NEAR(tTraj[i], tTraj_boost[i], 1e-6);
    EXPECT_TRUE(xTraj[i].isApprox(xTraj_boost[i], 1e-6));
  }
}

TEST(RungeKuttaDormandPrince5Test, IntegrateTimesCompareWithBoost) {
  const ocs2::scalar_t t0 = 0.0;
  const ocs2::scalar_t t1 = 10.0;
  const ocs2::scalar_t dt = 0.05;
  const ocs2::vector_t x0 = ocs2::vector_t::Zero(2);

  LinearSystem sys;

  ocs2::scalar_array_t times = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};

  ocs2::vector_array_t xTraj;
  ocs2::Observer observer(&xTraj);
  auto integrator = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);
  integrator->integrateTimes(sys, observer, x0, times.begin(), times.end(), dt);

  ocs2::vector_array_t xTraj_boost;
  ocs2::Observer observer_boost(&xTraj_boost);
  auto integrator_boost = ocs2::newIntegrator(ocs2::IntegratorType::ODE45);
  integrator_boost->integrateTimes(sys, observer_boost, x0, times.begin(), times.end(), dt);

  for (size_t i = 0; i < times.size(); i++) {
    EXPECT_TRUE(xTraj[i].isApprox(xTraj_boost[i], 1e-6));
  }
}

TEST(RungeKuttaDormandPrince5Test, integrateBackwards) {
  LinearSystem sys;
  auto integrator = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);

  const ocs2::scalar_t t0 = 0.0;
  const ocs2::scalar_t t1 = 10.0;
  const ocs2::scalar_t dt = 0.01;
  const ocs2::scalar_t absTol = 1e-9;
  const ocs2::scalar_t relTol = 1e-6;
  const ocs2::vector_t x0 = ocs2::vector_t::Zero(2);

  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory;
  auto observer = ocs2::Observer(&stateTrajectory, &timeTrajectory);

  // integrate forward from t0 to t1
  integrator->integrateAdaptive(sys, observer, x0, t0, t1, dt, absTol, relTol);

  const ocs2::vector_t x1 = stateTrajectory.back();
  stateTrajectory.clear();
  timeTrajectory.clear();
  observer = ocs2::Observer(&stateTrajectory, &timeTrajectory);

  // integrate backward from t1 to t0
  integrator->integrateAdaptive(sys, observer, x1, t1, t0, -dt, absTol, relTol);

  EXPECT_NEAR(timeTrajectory.front(), t1, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t0, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x1));
  // Choosing an appropriate tolerance is tricky
  EXPECT_TRUE((stateTrajectory.back() - x0).norm() < 1e-3);
}
