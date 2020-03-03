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
  /** First derivative of the cost w.r.t state */  // NOLINTNEXTLINE
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

/** Defines the quadratic cost and  linear dynamics at a give stage */
struct LinearQuadraticStage {
  /** Quadratic approximation of the cost */  // NOLINTNEXTLINE
  QuadraticCost cost;
  /** Linear approximation of the dynamics */  // NOLINTNEXTLINE
  LinearDynamics dynamics;

  LinearQuadraticStage() = default;
  LinearQuadraticStage(QuadraticCost c, LinearDynamics d) : cost(std::move(c)), dynamics(std::move(d)) {}
};

/**
 * Defines dimensions of the linear quadratic optimal control problem
 * Each stage can have a different amount of states and inputs
 */
struct ProblemDimensions {
  /** Number of time steps N */  // NOLINTNEXTLINE
  int numStages;
  /** Number of states at each point in time, size N+1 */  // NOLINTNEXTLINE
  std::vector<int> numStates;
  /** Number of inputs at each point in time, size N */  // NOLINTNEXTLINE
  std::vector<int> numInputs;

  explicit ProblemDimensions(int N) : ProblemDimensions(N, 0, 0) {}
  /**
   * Constructor for constant size state and inputs
   * @param N : number of stages
   * @param nx : number of states for all stages
   * @param nu : number of inputs for all stages
   */
  ProblemDimensions(int N, int nx, int nu) : numStages(N), numStates(N + 1, nx), numInputs(N, nu) {}
};

}  // namespace ocs2_qp_solver