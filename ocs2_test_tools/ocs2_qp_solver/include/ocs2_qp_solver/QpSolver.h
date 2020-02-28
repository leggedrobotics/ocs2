//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include "ocs2_qp_solver/QpSolverTypes.h"
#include "ocs2_qp_solver/QpTrajectories.h"

namespace ocs2_qp_solver {

/**
 * Solves the discretized linear quadratic optimal control problem by constructing a dense QP and inverting the full KKT system.
 * The decision vector is defined as w = [dx[0], du[0], dx[1],  du[1], ..., dx[N]]
 *
 * @param lqApproximation : vector of stage-wise discrete quadratic cost and linear dynamics
 * @param problemDimensions : state and input sizes of the problem
 * @param linearizationTrajectory : trajectory the lqApproximation was made around.
 * @param initialState : initial state (in absolute coordinates)
 * @return trajectory of state and inputs (in relative coordinates), .i.e. dx(t), du(t)
 */
ContinuousTrajectory solveLinearQuadraticApproximation(const std::vector<LinearQuadraticStage>& lqApproximation,
                                                       const ProblemDimensions& problemDimensions,
                                                       const ContinuousTrajectory& linearizationTrajectory,
                                                       const Eigen::VectorXd& initialState);

/**
 * Constructs the matrix of stacked dynamic constraints A w = b
 *
 * A = [ I  *
 *       A  B -I  *
 *       *  *  A  B -I  *
 *       *  *  *  *  A  B -I ]
 *
 * b = [x0; -b[0]; ... -b[N-1]]
 *
 * @param dims : dimensions of the linear quadratic problem.
 * @param lqp : linear quadratic problem.
 * @param dx0 : initial state deviation from the linearization.
 * @return {A, b}
 */
std::pair<Eigen::MatrixXd, Eigen::VectorXd> getConstraintMatrices(const ProblemDimensions& dims,
                                                                  const std::vector<LinearQuadraticStage>& lqp, const Eigen::VectorXd& dx0);

/**
 * Constructs a matrix of stacked cost functions  1/2 w' H w + g' w
 *
 * H = [ Q  P' *
 *       P  R  *
 *             Q  P'
 *             P  R ]
 *
 * g = [q[0]; r[0]; q[1]; r[1]; ... ]
 *
 * @param dims
 * @param lqp
 * @return {H, g}
 */
std::pair<Eigen::MatrixXd, Eigen::VectorXd> getCostMatrices(const ProblemDimensions& dims, const std::vector<LinearQuadraticStage>& lqp);

/**
 * Solves the equality constrained QP
 * min_w  1/2 w' H w + g' w
 *   s.t. A w = b
 *
 *   Assumes H is postive definite, rows of A are linearly independent.
 *
 * @return {w, lambda} at the solution, where lambda are the lagrange multipliers
 */
std::pair<Eigen::VectorXd, Eigen::VectorXd> solveDenseQp(const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& Hg,
                                                         const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& Ab);

/**
 * Reconstructs the optimal state and input trajectory recursively based on the full qp solution vector
 * @param dims : problem dimensions
 * @param w : full qp solution vector
 * @return { state_trajectory, input_trajectory }
 */
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> getStateAndInputTrajectory(const ProblemDimensions& dims,
                                                                                                 const Eigen::VectorXd& w);

}  // namespace ocs2_qp_solver
