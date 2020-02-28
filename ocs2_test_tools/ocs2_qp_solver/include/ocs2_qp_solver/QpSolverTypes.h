//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ocs2_qp_solver {

/**
 * Defines the quadratic cost  1/2 x' Q x + u' P x + 1/2 u' R u + q' x + r' u + c
 * x, and u are in relative coordinates when representing a quadratic approximation
 */
struct QuadraticCost {
  /** Second derivative of the cost w.r.t state */  // NOLINTNEXTLINE
  Eigen::MatrixXd Q;
  /** Second derivative of the cost w.r.t input (lhs) and state (rhs) */  // NOLINTNEXTLINE
  Eigen::MatrixXd P;
  /** Second derivative of the cost w.r.t input */  // NOLINTNEXTLINE
  Eigen::MatrixXd R;
  /** First derivative of the cost w.r.t input */  // NOLINTNEXTLINE
  Eigen::VectorXd q;
  /** First derivative of the cost w.r.t input */  // NOLINTNEXTLINE
  Eigen::VectorXd r;
  /** Constant cost term */  // NOLINTNEXTLINE
  double c = 0.;
};

/**
 * Defines the linear dynamics (continuous: dxdt = A x + B u + b) or (discrete: x[k+1] = A x[k] + B x[k] + b)
 * x, and u (on the rhs) are in relative coordinates when representing a linear approximation
 */
struct LinearDynamics {
  /** Derivative of the flowmap w.r.t state */  // NOLINTNEXTLINE
  Eigen::MatrixXd A;
  /** Derivative of the flowmap w.r.t input */  // NOLINTNEXTLINE
  Eigen::MatrixXd B;
  /** Flowmap bias */  // NOLINTNEXTLINE
  Eigen::VectorXd b;
};

}  // namespace ocs2_qp_solver