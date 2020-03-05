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
 * Generates a discrete time control problem with quadratic costs and affine dynamics.
 * The discretization stepsizes are defined by the time trajectory of the provided linearization trajectory.
 *
 * @param cost : continuous cost function
 * @param system : continuous system dynamics
 * @param linearizationTrajectory : time, state and input trajectory to make the linear quadratic approximation around
 * @return vector of discrete cost and dynamics at each node.
 */
std::vector<LinearQuadraticStage> getLinearQuadraticApproximation(CostWrapper& cost, SystemWrapper& system,
                                                                  const ContinuousTrajectory& linearizationTrajectory);

/**
 * Extracts the problem state and inputs dimensions from the a linear quadratic approximation
 * Looks at the size of the flowmap derivatives of the dynamics.
 * @return { numStatesPerStage, numInputsPerStage }
 */
std::pair<std::vector<int>, std::vector<int>> getNumStatesAndInputs(const std::vector<LinearQuadraticStage>& linearQuadraticApproximation);

/**
 * Constructs the discrete quadratic cost and linear dynamics between the given start and end conditions
 *
 * @param cost : continuous cost
 * @param system : continuous system
 * @param start : linearization point at the start of the stage
 * @param end : linearization point at the end of the stage
 * @return discreted stage
 */
LinearQuadraticStage discretizeStage(CostWrapper& cost, SystemWrapper& system, TrajectoryRef start, StateTrajectoryRef end);

/**
 * Computes the cost integral from a start condition over a dt interval
 * @param cost : continuous cost
 * @param start : linearization point at the start of the interval
 * @param dt : duration of the interval
 * @return Quadratic approximation of the accumulated costs
 */
QuadraticCost discretizeCost(CostWrapper& cost, TrajectoryRef start, double dt);

/**
 * Computes the discrete dynamics from a start condition over a dt interval
 * @param system : continuous system
 * @param start : linearization point at the start of the interval
 * @param dt : duration of the interval
 * @return
 */
LinearDynamics discretizeDynamics(SystemWrapper& system, TrajectoryRef start, double dt);

}  // namespace qp_solver
}  // namespace ocs2
