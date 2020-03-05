//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include "ocs2_qp_solver/QpSolverTypes.h"
#include "ocs2_qp_solver/QpTrajectories.h"
#include "ocs2_qp_solver/wrappers/CostWrapper.h"
#include "ocs2_qp_solver/wrappers/SystemWrapper.h"

namespace ocs2 {
namespace qp_solver {

/**
 * Solves a discrete time linear quadratic control problem around a provided linearization trajectory.
 * The time horizon and discretization steps are defined by the time trajectory of the provided linearization.
 *
 * @param cost : continuous cost function
 * @param system : continuous system dynamics
 * @param linearizationTrajectory : time, state and input trajectory to make the linear quadratic approximation around
 * @param initialState : state at the start of the horizon.
 * @return time, state, and input solution.
 */
ContinuousTrajectory solveLinearQuadraticOptimalControlProblem(CostWrapper costFunction, SystemWrapper systemDynamics,
                                                               const ContinuousTrajectory& linearizationTrajectory,
                                                               const Eigen::VectorXd& initialState);

}  // namespace qp_solver
}  // namespace ocs2
