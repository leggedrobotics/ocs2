//
// Created by rgrandia on 25.02.20.
//

#include "ocs2_qp_solver/Ocs2QpSolver.h"

#include "ocs2_qp_solver/QpDiscreteTranscription.h"
#include "ocs2_qp_solver/QpSolver.h"

namespace ocs2 {
namespace qp_solver {

ContinuousTrajectory solveLinearQuadraticOptimalControlProblem(CostWrapper costFunction, SystemWrapper systemDynamics,
                                                               const ContinuousTrajectory& linearizationTrajectory,
                                                               const Eigen::VectorXd& initialState) {
  // Approximate
  const auto lqApproximation = getLinearQuadraticApproximation(costFunction, systemDynamics, linearizationTrajectory);
  const auto problemDimensions = getProblemDimensions(lqApproximation);

  // Solve for update step
  const auto relativeSolution =
      solveLinearQuadraticApproximation(lqApproximation, problemDimensions, linearizationTrajectory, initialState);

  // Take a full step: Add update to linearization
  return add(linearizationTrajectory, relativeSolution);
}

}  // namespace qp_solver
}  // namespace ocs2