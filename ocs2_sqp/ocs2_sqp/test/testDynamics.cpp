//
// Created by rgrandia on 16.03.21.
//

#include <gtest/gtest.h>

#include "ocs2_sqp/DynamicsDiscretization.h"

#include <ocs2_oc/test/circular_kinematics.h>

TEST(test_dynamics, rk4Sensitivity) {
  ocs2::CircularKinematicsSystem system;
  ocs2::scalar_t t = 0.5;
  ocs2::vector_t x = ocs2::vector_t::Random(2);
  ocs2::vector_t u = ocs2::vector_t::Random(2);
  ocs2::scalar_t dt = 0.0;

  // Check with more readable version of sensitivity computation.
  const auto rk4dynamics_check = [&]() {
    const ocs2::scalar_t dt_halve = dt / 2.0;
    const ocs2::scalar_t dt_sixth = dt / 6.0;
    const ocs2::scalar_t dt_third = dt / 3.0;

    // System evaluations
    const ocs2::VectorFunctionLinearApproximation k1 = system.linearApproximation(t, x, u);
    const ocs2::VectorFunctionLinearApproximation k2 = system.linearApproximation(t + dt_halve, x + dt_halve * k1.f, u);
    const ocs2::VectorFunctionLinearApproximation k3 = system.linearApproximation(t + dt_halve, x + dt_halve * k2.f, u);
    const ocs2::VectorFunctionLinearApproximation k4 = system.linearApproximation(t + dt, x + dt * k3.f, u);

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

  const auto rk4Dynamics = rk4SensitivityDiscretization(system, t, x, u, dt);
  ASSERT_TRUE(rk4Dynamics.f.isApprox(rk4dynamics_check.f));
  ASSERT_TRUE(rk4Dynamics.dfdx.isApprox(rk4dynamics_check.dfdx));
  ASSERT_TRUE(rk4Dynamics.dfdu.isApprox(rk4dynamics_check.dfdu));
}