//
// Created by rgrandia on 18.02.21.
//

#include "ocs2_sqp/DynamicsDiscretization.h"

namespace ocs2 {

VectorFunctionLinearApproximation eulerDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                      scalar_t dt) {
  // x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  // A_{k} = Id + dt * dfdx
  // B_{k} = dt * dfdu
  // b_{k} = x_{n} + dt * f(x_{n},u_{n})
  VectorFunctionLinearApproximation continuousApproximation = system.linearApproximation(t, x, u);
  VectorFunctionLinearApproximation discreteApproximation;
  const int nx = x.rows();
  discreteApproximation.dfdx = matrix_t::Identity(nx, nx) + dt * continuousApproximation.dfdx;
  discreteApproximation.dfdu = dt * continuousApproximation.dfdu;
  discreteApproximation.f = x + dt * continuousApproximation.f;
  return discreteApproximation;
}

VectorFunctionLinearApproximation rk4Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                    scalar_t dt) {
  const int nx = x.rows();
  const scalar_t dt_halve = dt / 2.0;
  const scalar_t dt_sixth = dt / 6.0;
  const scalar_t dt_third = dt / 3.0;

  // System evaluations
  const ocs2::VectorFunctionLinearApproximation k1 = system.linearApproximation(t, x, u);
  const ocs2::VectorFunctionLinearApproximation k2 = system.linearApproximation(t + dt_halve, x + dt_halve * k1.f, u);
  const ocs2::VectorFunctionLinearApproximation k3 = system.linearApproximation(t + dt_halve, x + dt_halve * k2.f, u);
  const ocs2::VectorFunctionLinearApproximation k4 = system.linearApproximation(t + dt, x + dt * k3.f, u);

  // Sensitivity propagation
  const matrix_t dk1dxk = k1.dfdx;
  const matrix_t dk2dxk = k2.dfdx * (matrix_t::Identity(nx, nx) + dt_halve * dk1dxk);
  const matrix_t dk3dxk = k3.dfdx * (matrix_t::Identity(nx, nx) + dt_halve * dk2dxk);
  const matrix_t dk4dxk = k4.dfdx * (matrix_t::Identity(nx, nx) + dt * dk3dxk);
  const matrix_t dk1duk = k1.dfdu;
  const matrix_t dk2duk = k2.dfdu + dt_halve * k2.dfdx * dk1duk;
  const matrix_t dk3duk = k3.dfdu + dt_halve * k3.dfdx * dk2duk;
  const matrix_t dk4duk = k4.dfdu + dt * k4.dfdx * dk3duk;

  // Assemble discrete approximation
  VectorFunctionLinearApproximation discreteApproximation;
  discreteApproximation.dfdx = matrix_t::Identity(nx, nx) + dt_sixth * dk1dxk + dt_third * dk2dxk + dt_third * dk3dxk + dt_sixth * dk4dxk;
  //  matrix_t trueBMatrix =
  //      (dt / 6.0) * (dk1duk + 2 * dk2duk + 2 * dk3duk + dk4duk);              // <-- this should be the correct one, but not working
  discreteApproximation.dfdu =
      dt_sixth * k1.dfdu + dt_third * k2.dfdu + dt_third * k3.dfdu + dt_sixth * k4.dfdu;  // <-- this is working but not the right one
  //  std::cout << "difference of B and true B " << (trueBMatrix - discreteApproximation.dfdu).norm() << std::endl;
  // dynamics_[i].dfdu = trueBMatrix;
  discreteApproximation.f = x + dt_sixth * k1.f + dt_third * k2.f + dt_third * k3.f + dt_sixth * k4.f;
  return discreteApproximation;
}

}  // namespace ocs2