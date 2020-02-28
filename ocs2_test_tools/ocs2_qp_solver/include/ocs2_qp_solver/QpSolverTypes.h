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
  Eigen::MatrixXd Q;
  Eigen::MatrixXd P;
  Eigen::MatrixXd R;
  Eigen::VectorXd q;
  Eigen::VectorXd r;
  double c;
};

/**
 * Defines the linear dynamics (continuous: dxdt = A x + B u + b) or (discrete: x[k+1] = A x[k] + B x[k] + b)
 * x, and u (on the rhs) are in relative coordinates when representing a linear approximation
 */
struct LinearDynamics {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd b;
};

/** Defines the quadratic cost and  linear dynamics at a give stage */
struct LinearQuadraticStage {
  QuadraticCost cost;
  LinearDynamics dynamics;

  LinearQuadraticStage() = default;
  LinearQuadraticStage(QuadraticCost c, LinearDynamics d) : cost(std::move(c)), dynamics(std::move(d)) {}
};

/**
 * Defines dimensions of the linear quadratic optimal control problem
 * Each stage can have a different amount of states and inputs
 */
struct ProblemDimensions {
  int numStages;               // N
  std::vector<int> numStates;  // size N+1
  std::vector<int> numInputs;  // size N

  ProblemDimensions() = default;
  /**
   * Constructor for constant size state and inputs
   * @param N : number of stages
   * @param nx : number of states for all stages
   * @param nu : number of inputs for all stages
   */
  ProblemDimensions(int N, int nx, int nu) : numStages(N), numStates(N + 1, nx), numInputs(N, nu) {}
};

}  // namespace ocs2_qp_solver