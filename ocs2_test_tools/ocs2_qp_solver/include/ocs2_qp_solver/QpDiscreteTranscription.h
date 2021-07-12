/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include "ocs2_qp_solver/QpSolverTypes.h"
#include "ocs2_qp_solver/QpTrajectories.h"

namespace ocs2 {
namespace qp_solver {

/**
 * Generates a discrete time control problem with quadratic costs and affine dynamics.
 * The discretization stepsizes are defined by the time trajectory of the provided linearization trajectory.
 *
 * @param cost : continuous cost function
 * @param system : continuous system dynamics
 * @param constraintsPtr : constraints. For unconstrained problems use a null pointer.
 * @param nominalTrajectory : time, state and input trajectory to make the linear quadratic approximation around
 * @return vector of discrete cost and dynamics at each node.
 */
std::vector<LinearQuadraticStage> getLinearQuadraticApproximation(CostFunctionBase& cost, SystemDynamicsBase& system,
                                                                  ConstraintBase* constraintsPtr,
                                                                  const ContinuousTrajectory& nominalTrajectory);

/**
 * Constructs the discrete quadratic cost and linear dynamics between the given start and end conditions
 *
 * @param cost : continuous cost
 * @param system : continuous system
 * @param constraintsPtr : constraints. For unconstrained problems use a null pointer.
 * @param start : linearization point at the start of the stage
 * @param end : linearization point at the end of the stage
 * @param isInitialTime : Whether start is an initial time points.
 * @return discreted stage
 */
LinearQuadraticStage approximateStage(CostFunctionBase& cost, SystemDynamicsBase& system, ConstraintBase* constraintsPtr,
                                      TrajectoryRef start, StateTrajectoryRef end, bool isInitialTime);

/**
 * Computes the cost integral from a start condition over a dt interval
 * @param cost : continuous cost
 * @param start : linearization point at the start of the interval
 * @param dt : duration of the interval
 * @return Quadratic approximation of the accumulated costs
 */
ScalarFunctionQuadraticApproximation approximateCost(CostFunctionBase& cost, TrajectoryRef start, scalar_t dt);

/**
 * Computes the discrete dynamics from a start condition over a dt interval
 * @param system : continuous system
 * @param start : linearization point at the start of the interval
 * @param dt : duration of the interval
 * @return Linear approximation of the discrete dynamcis
 */
VectorFunctionLinearApproximation approximateDynamics(SystemDynamicsBase& system, TrajectoryRef start, scalar_t dt);

/**
 * Computes the equality constraints at given trajectory point
 * @param constraints : constraints object
 * @param point : linearization point
 * @return Linear approximation of the constraints
 */
VectorFunctionLinearApproximation approximateConstraints(ConstraintBase& constraints, TrajectoryRef point, bool isInitialTime);

}  // namespace qp_solver
}  // namespace ocs2
