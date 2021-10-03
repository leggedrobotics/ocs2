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

#include "ocs2_core/integration/Integrator.h"
#include "ocs2_core/integration/SensitivityIntegrator.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace {
std::unique_ptr<ocs2::LinearSystemDynamics> getSystem() {
  ocs2::matrix_t A(2, 2);
  A << -2, -1,  // clang-format off
      1,  0;  // clang-format on
  ocs2::matrix_t B(2, 1);
  B << 1, 0;
  return std::unique_ptr<ocs2::LinearSystemDynamics>(new ocs2::LinearSystemDynamics(A, B));
}
}  // namespace

TEST(test_sensitivity_integrator, eulerSensitivity) {
  auto type = ocs2::SensitivityIntegratorType::EULER;
  auto eulerSensitivityDiscretization = ocs2::selectDynamicsSensitivityDiscretization(type);
  auto eulerDiscretization = ocs2::selectDynamicsDiscretization(type);

  auto system = getSystem();
  ocs2::scalar_t t = 0.5;
  ocs2::vector_t x = ocs2::vector_t::Random(2);
  ocs2::vector_t u = ocs2::vector_t::Random(1);
  ocs2::scalar_t dt = 0.1;

  // Check with more readable version of sensitivity computation.
  const auto eulerdynamics_check = [&]() {
    const ocs2::PreComputation preComp;

    // System evaluations
    const ocs2::VectorFunctionLinearApproximation k1 = system->linearApproximation(t, x, u, preComp);

    // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
    const ocs2::matrix_t dk1dxk = k1.dfdx;

    // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
    const ocs2::matrix_t dk1duk = k1.dfdu;

    // Assemble discrete approximation
    ocs2::VectorFunctionLinearApproximation discreteApproximation;
    discreteApproximation.dfdx = ocs2::matrix_t::Identity(x.size(), x.size()) + dt * dk1dxk;
    discreteApproximation.dfdu = dt * dk1duk;
    discreteApproximation.f = x + dt * k1.f;
    return discreteApproximation;
  }();

  const auto eulerForwardDynamics = eulerDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(eulerForwardDynamics.isApprox(eulerdynamics_check.f));
  const auto eulerLinearizedDynamics = eulerSensitivityDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(eulerLinearizedDynamics.f.isApprox(eulerdynamics_check.f));
  ASSERT_TRUE(eulerLinearizedDynamics.dfdx.isApprox(eulerdynamics_check.dfdx));
  ASSERT_TRUE(eulerLinearizedDynamics.dfdu.isApprox(eulerdynamics_check.dfdu));
}

TEST(test_sensitivity_integrator, rk2Sensitivity) {
  auto type = ocs2::SensitivityIntegratorType::RK2;
  auto rk2SensitivityDiscretization = ocs2::selectDynamicsSensitivityDiscretization(type);
  auto rk2Discretization = ocs2::selectDynamicsDiscretization(type);

  auto system = getSystem();
  ocs2::scalar_t t = 0.5;
  ocs2::vector_t x = ocs2::vector_t::Random(2);
  ocs2::vector_t u = ocs2::vector_t::Random(1);
  ocs2::scalar_t dt = 0.1;

  // Check with more readable version of sensitivity computation.
  const auto rk2dynamics_check = [&]() {
    const ocs2::scalar_t dt_halve = dt / 2.0;
    const ocs2::PreComputation preComp;

    // System evaluations
    const ocs2::VectorFunctionLinearApproximation k1 = system->linearApproximation(t, x, u, preComp);
    const ocs2::VectorFunctionLinearApproximation k2 = system->linearApproximation(t + dt, x + dt * k1.f, u, preComp);

    // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
    const ocs2::matrix_t dk1dxk = k1.dfdx;
    const ocs2::matrix_t dk2dxk = dt * k2.dfdx * dk1dxk + k2.dfdx;

    // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
    const ocs2::matrix_t dk1duk = k1.dfdu;
    const ocs2::matrix_t dk2duk = dt * k2.dfdx * dk1duk + k2.dfdu;

    // Assemble discrete approximation
    ocs2::VectorFunctionLinearApproximation discreteApproximation;
    discreteApproximation.dfdx = ocs2::matrix_t::Identity(x.size(), x.size()) + dt_halve * dk1dxk + dt_halve * dk2dxk;
    discreteApproximation.dfdu = dt_halve * dk1duk + dt_halve * dk2duk;
    discreteApproximation.f = x + dt_halve * k1.f + dt_halve * k2.f;
    return discreteApproximation;
  }();

  const auto rk2ForwardDynamics = rk2Discretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk2ForwardDynamics.isApprox(rk2dynamics_check.f));
  const auto rk2LinearizedDynamics = rk2SensitivityDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk2LinearizedDynamics.f.isApprox(rk2dynamics_check.f));
  ASSERT_TRUE(rk2LinearizedDynamics.dfdx.isApprox(rk2dynamics_check.dfdx));
  ASSERT_TRUE(rk2LinearizedDynamics.dfdu.isApprox(rk2dynamics_check.dfdu));
}

TEST(test_sensitivity_integrator, rk4Sensitivity) {
  auto type = ocs2::SensitivityIntegratorType::RK4;
  auto rk4SensitivityDiscretization = ocs2::selectDynamicsSensitivityDiscretization(type);
  auto rk4Discretization = ocs2::selectDynamicsDiscretization(type);

  auto system = getSystem();
  ocs2::scalar_t t = 0.5;
  ocs2::vector_t x = ocs2::vector_t::Random(2);
  ocs2::vector_t u = ocs2::vector_t::Random(1);
  ocs2::scalar_t dt = 0.1;

  // Check with more readable version of sensitivity computation.
  const auto rk4dynamics_check = [&]() {
    const ocs2::scalar_t dt_halve = dt / 2.0;
    const ocs2::scalar_t dt_sixth = dt / 6.0;
    const ocs2::scalar_t dt_third = dt / 3.0;
    const ocs2::PreComputation preComp;

    // System evaluations
    const ocs2::VectorFunctionLinearApproximation k1 = system->linearApproximation(t, x, u, preComp);
    const ocs2::VectorFunctionLinearApproximation k2 = system->linearApproximation(t + dt_halve, x + dt_halve * k1.f, u, preComp);
    const ocs2::VectorFunctionLinearApproximation k3 = system->linearApproximation(t + dt_halve, x + dt_halve * k2.f, u, preComp);
    const ocs2::VectorFunctionLinearApproximation k4 = system->linearApproximation(t + dt, x + dt * k3.f, u, preComp);

    // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
    const ocs2::matrix_t dk1dxk = k1.dfdx;
    const ocs2::matrix_t dk2dxk = dt_halve * k2.dfdx * dk1dxk + k2.dfdx;
    const ocs2::matrix_t dk3dxk = dt_halve * k3.dfdx * dk2dxk + k3.dfdx;
    const ocs2::matrix_t dk4dxk = dt * k4.dfdx * dk3dxk + k4.dfdx;

    // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
    const ocs2::matrix_t dk1duk = k1.dfdu;
    const ocs2::matrix_t dk2duk = dt_halve * k2.dfdx * dk1duk + k2.dfdu;
    const ocs2::matrix_t dk3duk = dt_halve * k3.dfdx * dk2duk + k3.dfdu;
    const ocs2::matrix_t dk4duk = dt * k4.dfdx * dk3duk + k4.dfdu;

    // Assemble discrete approximation
    ocs2::VectorFunctionLinearApproximation discreteApproximation;
    discreteApproximation.dfdx =
        ocs2::matrix_t::Identity(x.size(), x.size()) + dt_sixth * dk1dxk + dt_third * dk2dxk + dt_third * dk3dxk + dt_sixth * dk4dxk;
    discreteApproximation.dfdu = dt_sixth * dk1duk + dt_third * dk2duk + dt_third * dk3duk + dt_sixth * dk4duk;
    discreteApproximation.f = x + dt_sixth * k1.f + dt_third * k2.f + dt_third * k3.f + dt_sixth * k4.f;
    return discreteApproximation;
  }();

  const auto rk4ForwardDynamics = rk4Discretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk4ForwardDynamics.isApprox(rk4dynamics_check.f));
  const auto rk4LinearizedDynamics = rk4SensitivityDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk4LinearizedDynamics.f.isApprox(rk4dynamics_check.f));
  ASSERT_TRUE(rk4LinearizedDynamics.dfdx.isApprox(rk4dynamics_check.dfdx));
  ASSERT_TRUE(rk4LinearizedDynamics.dfdu.isApprox(rk4dynamics_check.dfdu));
}

TEST(test_sensitivity_integrator, vsBoostRK4) {
  auto system = getSystem();
  ocs2::scalar_t t = 0.5;
  ocs2::vector_t x = ocs2::vector_t::Random(2);
  ocs2::vector_t u = ocs2::vector_t::Random(1);
  ocs2::scalar_t dt = 0.1;

  // Boost version
  auto integrator = newIntegrator(ocs2::IntegratorType::RK4);
  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory;
  auto observer = ocs2::Observer(&stateTrajectory, &timeTrajectory);
  ocs2::FeedforwardController controller({t, t}, {u, u});
  system->setController(&controller);
  int maxNumSteps = 4;
  integrator->integrateConst(*system, observer, x, t, t + dt, dt, maxNumSteps);
  const auto boostRk4ForwardDynamics = stateTrajectory.back();

  // This version
  auto type = ocs2::SensitivityIntegratorType::RK4;
  auto rk4Discretization = ocs2::selectDynamicsDiscretization(type);
  const auto rk4ForwardDynamics = rk4Discretization(*system, t, x, u, dt);

  // Check
  ASSERT_TRUE(rk4ForwardDynamics.isApprox(boostRk4ForwardDynamics));
}