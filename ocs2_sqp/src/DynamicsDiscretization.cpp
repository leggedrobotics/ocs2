//
// Created by rgrandia on 18.02.21.
//

#include "ocs2_sqp/DynamicsDiscretization.h"

namespace ocs2 {

vector_t eulerDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt) {
  return x + dt * system.computeFlowMap(t, x, u);
}

VectorFunctionLinearApproximation eulerSensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x,
                                                                 const vector_t& u, scalar_t dt) {
  // x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  // A_{k} = Id + dt * dfdx
  // B_{k} = dt * dfdu
  // b_{k} = x_{n} + dt * f(x_{n},u_{n})
  const auto continuousApproximation = system.linearApproximation(t, x, u);
  VectorFunctionLinearApproximation discreteApproximation;
  discreteApproximation.dfdx = dt * continuousApproximation.dfdx;
  discreteApproximation.dfdx.diagonal().array() += 1.0;  // plus Identity()
  discreteApproximation.dfdu = dt * continuousApproximation.dfdu;
  discreteApproximation.f = x + dt * continuousApproximation.f;
  return discreteApproximation;
}

vector_t rk4Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt) {
  const scalar_t dt_halve = dt / 2.0;
  const scalar_t dt_sixth = dt / 6.0;
  const scalar_t dt_third = dt / 3.0;

  // System evaluations
  const vector_t k1 = system.computeFlowMap(t, x, u);
  const vector_t k2 = system.computeFlowMap(t + dt_halve, x + dt_halve * k1, u);
  const vector_t k3 = system.computeFlowMap(t + dt_halve, x + dt_halve * k2, u);
  const vector_t k4 = system.computeFlowMap(t + dt, x + dt * k3, u);

  return x + dt_sixth * k1 + dt_third * k2 + dt_third * k3 + dt_sixth * k4;
}

VectorFunctionLinearApproximation rk4SensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                               scalar_t dt) {
  const scalar_t dt_halve = dt / 2.0;
  const scalar_t dt_sixth = dt / 6.0;
  const scalar_t dt_third = dt / 3.0;

  // System evaluations
  const VectorFunctionLinearApproximation k1 = system.linearApproximation(t, x, u);
  const VectorFunctionLinearApproximation k2 = system.linearApproximation(t + dt_halve, x + dt_halve * k1.f, u);
  const VectorFunctionLinearApproximation k3 = system.linearApproximation(t + dt_halve, x + dt_halve * k2.f, u);
  const VectorFunctionLinearApproximation k4 = system.linearApproximation(t + dt, x + dt * k3.f, u);

  // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
  const auto& dk1dxk = k1.dfdx;
  matrix_t dk2dxk = k2.dfdx;
  dk2dxk.noalias() += dt_halve * k2.dfdx * dk1dxk;
  matrix_t dk3dxk = k3.dfdx;
  dk3dxk.noalias() += dt_halve * k3.dfdx * dk2dxk;
  matrix_t dk4dxk = k4.dfdx;
  dk4dxk.noalias() += dt * k4.dfdx * dk3dxk;

  // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
  const auto& dk1duk = k1.dfdu;
  matrix_t dk2duk = k2.dfdu;
  dk2duk.noalias() += dt_halve * k2.dfdx * dk1duk;
  matrix_t dk3duk = k3.dfdu;
  dk3duk.noalias() += dt_halve * k3.dfdx * dk2duk;
  matrix_t dk4duk = k4.dfdu;
  dk4duk.noalias() += dt * k4.dfdx * dk3duk;

  // Assemble discrete approximation
  VectorFunctionLinearApproximation discreteApproximation;
  discreteApproximation.dfdx = dt_sixth * dk1dxk + dt_third * dk2dxk + dt_third * dk3dxk + dt_sixth * dk4dxk;
  discreteApproximation.dfdx.diagonal().array() += 1.0;  // plus Identity()
  discreteApproximation.dfdu = dt_sixth * dk1duk + dt_third * dk2duk + dt_third * dk3duk + dt_sixth * dk4duk;
  discreteApproximation.f = x + dt_sixth * k1.f + dt_third * k2.f + dt_third * k3.f + dt_sixth * k4.f;
  return discreteApproximation;
}

}  // namespace ocs2