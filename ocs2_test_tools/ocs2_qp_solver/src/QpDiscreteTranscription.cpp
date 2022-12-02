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

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

#include "ocs2_qp_solver/QpDiscreteTranscription.h"

namespace ocs2 {
namespace qp_solver {

std::vector<LinearQuadraticStage> getLinearQuadraticApproximation(OptimalControlProblem& optimalControProblem,
                                                                  const ContinuousTrajectory& nominalTrajectory) {
  // OCP check
  if (!optimalControProblem.equalityLagrangianPtr->empty() || !optimalControProblem.stateEqualityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] equalityLagrangianPtr and stateEqualityLagrangianPtr should be empty!");
  }
  if (!optimalControProblem.inequalityLagrangianPtr->empty() || !optimalControProblem.stateInequalityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] inequalityLagrangianPtr and stateInequalityLagrangianPtr should be empty!");
  }
  if (!optimalControProblem.finalEqualityLagrangianPtr->empty() || !optimalControProblem.finalInequalityLagrangianPtr->empty()) {
    throw std::runtime_error(
        "[getLinearQuadraticApproximation] finalEqualityLagrangianPtr and finalInequalityLagrangianPtr should be empty!");
  }

  if (nominalTrajectory.timeTrajectory.empty()) {
    return {};
  }

  auto& t = nominalTrajectory.timeTrajectory;
  auto& x = nominalTrajectory.stateTrajectory;
  auto& u = nominalTrajectory.inputTrajectory;
  const int N = t.size() - 1;

  // LinearQuadraticProblem with N+1 elements. Terminal stage lqp[N].dynamics is ignored.
  std::vector<LinearQuadraticStage> lqp;
  lqp.reserve(N + 1);
  for (int k = 0; k < N; ++k) {  // Intermediate stages
    lqp.emplace_back(approximateStage(optimalControProblem, {t[k], x[k], u[k]}, {t[k + 1], x[k + 1]}, k == 0));
  }

  auto modelData = approximateFinalLQ(optimalControProblem, t[N], x[N], MultiplierCollection());

  // checking the numerical properties
  const auto errSize = checkSize(modelData, x[N].rows(), 0);
  if (!errSize.empty()) {
    throw std::runtime_error("[qp_solver::getLinearQuadraticApproximation] Ill-posed problem at final time: " + std::to_string(t[N]) +
                             "\n" + errSize);
  }
  const std::string errProperties = checkCostProperties(modelData) + checkConstraintProperties(modelData);
  if (!errProperties.empty()) {
    throw std::runtime_error("[qp_solver::getLinearQuadraticApproximation] Ill-posed problem at final time: " + std::to_string(t[N]) +
                             "\n" + errProperties);
  }

  lqp.emplace_back(std::move(modelData.cost), VectorFunctionLinearApproximation(), std::move(modelData.stateEqConstraint));

  return lqp;
}

LinearQuadraticStage approximateStage(OptimalControlProblem& optimalControProblem, TrajectoryRef start, StateTrajectoryRef end,
                                      bool isInitialTime) {
  const auto modelData = approximateIntermediateLQ(optimalControProblem, start.t, start.x, start.u, MultiplierCollection());

  // checking the numerical properties
  const auto errSize = checkSize(modelData, start.x.rows(), start.u.rows());
  if (!errSize.empty()) {
    throw std::runtime_error("[[qp_solver::approximateStage] Ill-posed problem at intermediate time: " + std::to_string(start.t) + "\n" +
                             errSize);
  }
  const std::string errProperties =
      checkDynamicsProperties(modelData) + checkCostProperties(modelData) + checkConstraintProperties(modelData);
  if (!errProperties.empty()) {
    throw std::runtime_error("[qp_solver::approximateStage] Ill-posed problem at intermediate time: " + std::to_string(start.t) + "\n" +
                             errProperties);
  }

  LinearQuadraticStage lqStage;
  const auto dt = end.t - start.t;

  lqStage.cost = modelData.cost;
  lqStage.cost *= dt;

  // Linearized Dynamics after discretization: x0[k+1] + dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k])
  lqStage.dynamics = approximateDynamics(modelData, start, dt);
  // Adapt the offset to account for discretization and the nominal trajectory :
  // dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k]) - x0[k+1]
  lqStage.dynamics.f -= end.x;

  lqStage.constraints = approximateConstraints(modelData, isInitialTime);

  return lqStage;
}

VectorFunctionLinearApproximation approximateDynamics(const ModelData& modelData, TrajectoryRef start, scalar_t dt) {
  // Forward Euler discretization
  // x[k+1] = x[k] + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * (A_c dx[k] + B_c du[k] + b_c)
  // x[k+1] = (I + A_c * dt) dx[k] + (B_c * dt) du[k] + (b_c * dt + x0[k])
  const auto& continuousDynamics = modelData.dynamics;
  VectorFunctionLinearApproximation discreteDynamics;
  discreteDynamics.dfdx = continuousDynamics.dfdx * dt;
  discreteDynamics.dfdx.diagonal().array() += 1.0;
  discreteDynamics.dfdu = continuousDynamics.dfdu * dt;
  discreteDynamics.f = continuousDynamics.f * dt + start.x;
  return discreteDynamics;
}

VectorFunctionLinearApproximation approximateConstraints(const ModelData& modelData, bool isInitialTime) {
  VectorFunctionLinearApproximation constraintsApproximation;
  if (isInitialTime) {
    // only use stat-input constraints for initial time
    constraintsApproximation = std::move(modelData.stateInputEqConstraint);
  } else {
    // concatenate stat-input and state-only constraints
    const auto& stateInputConstraint = modelData.stateInputEqConstraint;
    const auto& stateConstraint = modelData.stateEqConstraint;
    const auto numStateInputConstraints = stateInputConstraint.f.size();
    const auto numStateConstraints = stateConstraint.f.size();
    const auto numStates = stateInputConstraint.dfdx.cols();
    const auto numInputs = stateInputConstraint.dfdu.cols();
    constraintsApproximation.resize(numStateConstraints + numStateInputConstraints, numStates, numInputs);
    constraintsApproximation.f.head(numStateInputConstraints) = stateInputConstraint.f;
    constraintsApproximation.dfdx.topRows(numStateInputConstraints) = stateInputConstraint.dfdx;
    constraintsApproximation.dfdu.topRows(numStateInputConstraints) = stateInputConstraint.dfdu;
    constraintsApproximation.f.tail(numStateConstraints) = stateConstraint.f;
    constraintsApproximation.dfdx.bottomRows(numStateConstraints) = stateConstraint.dfdx;
    constraintsApproximation.dfdu.bottomRows(numStateConstraints).setZero();
  }
  return constraintsApproximation;
}

}  // namespace qp_solver
}  // namespace ocs2
