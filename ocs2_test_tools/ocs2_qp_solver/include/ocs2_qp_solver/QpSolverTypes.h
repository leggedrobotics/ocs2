//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ocs2 {
namespace qp_solver {

/**
 * Defines the quadratic approximation f(x,u) = 1/2 dx' dfdxx dx + du' dfdux dx + 1/2 du' dfduu du + dfdx' dx + dfdu' du + f
 */
struct ScalarFunctionQuadraticApproximation {
  /** Second derivative w.r.t state */
  Eigen::MatrixXd dfdxx;
  /** Second derivative w.r.t input (lhs) and state (rhs) */
  Eigen::MatrixXd dfdux;
  /** Second derivative w.r.t input */
  Eigen::MatrixXd dfduu;
  /** First derivative w.r.t state */
  Eigen::VectorXd dfdx;
  /** First derivative w.r.t input */
  Eigen::VectorXd dfdu;
  /** Constant term */
  double f = 0.;
};

/**
 * Defines the linear model of a vector function f(x,u) = dfdx * dx + dfdu * du + df
 */
struct VectorFunctionLinearApproximation {
  /** Derivative w.r.t state */
  Eigen::MatrixXd dfdx;
  /** Derivative w.r.t input */
  Eigen::MatrixXd dfdu;
  /** Constant term */
  Eigen::VectorXd f;
};

/** Defines the quadratic cost and  linear dynamics at a give stage */
struct LinearQuadraticStage {
  /** Quadratic approximation of the cost */
  ScalarFunctionQuadraticApproximation cost;
  /** Linear approximation of the dynamics */
  VectorFunctionLinearApproximation dynamics;

  LinearQuadraticStage() = default;
  LinearQuadraticStage(ScalarFunctionQuadraticApproximation c, VectorFunctionLinearApproximation d)
      : cost(std::move(c)), dynamics(std::move(d)) {}
};

}  // namespace qp_solver
}  // namespace ocs2
