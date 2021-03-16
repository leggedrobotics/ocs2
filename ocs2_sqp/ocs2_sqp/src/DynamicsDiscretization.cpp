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
  VectorFunctionLinearApproximation k1 = system.linearApproximation(t, x, u);
  VectorFunctionLinearApproximation k2 = system.linearApproximation(t + dt_halve, x + dt_halve * k1.f, u);
  VectorFunctionLinearApproximation k3 = system.linearApproximation(t + dt_halve, x + dt_halve * k2.f, u);
  VectorFunctionLinearApproximation k4 = system.linearApproximation(t + dt, x + dt * k3.f, u);

  // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
  // Re-use memory from k.dfdu as dkduk
  // dk1duk = k1.dfdu
  k2.dfdu.noalias() += dt_halve * k2.dfdx * k1.dfdu;
  k3.dfdu.noalias() += dt_halve * k3.dfdx * k2.dfdu;
  k4.dfdu.noalias() += dt * k4.dfdx * k3.dfdu;

  // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
  // Re-use memory from k.dfdx as dkdxk
  // dk1dxk = k1.dfdx;
  matrix_t tmp = dt_halve * k2.dfdx * k1.dfdx;  // need one temporary to avoid alias
  k2.dfdx += tmp;
  tmp.noalias() = dt_halve * k3.dfdx * k2.dfdx;
  k3.dfdx += tmp;
  tmp.noalias() = dt * k4.dfdx * k3.dfdx;
  k4.dfdx += tmp;

  // Assemble discrete approximation
  // Re-use k1 to collect the result
  k1.dfdx = dt_sixth * k1.dfdx + dt_third * k2.dfdx + dt_third * k3.dfdx + dt_sixth * k4.dfdx;
  k1.dfdx.diagonal().array() += 1.0;  // plus Identity()
  k1.dfdu = dt_sixth * k1.dfdu + dt_third * k2.dfdu + dt_third * k3.dfdu + dt_sixth * k4.dfdu;
  k1.f = x + dt_sixth * k1.f + dt_third * k2.f + dt_third * k3.f + dt_sixth * k4.f;
  return k1;
}

}  // namespace ocs2