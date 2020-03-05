//
// Created by rgrandia on 26.02.20.
//

#include "ocs2_qp_solver/QpDiscreteTranscription.h"

namespace ocs2 {
namespace qp_solver {

std::vector<LinearQuadraticStage> getLinearQuadraticApproximation(CostWrapper& cost, SystemWrapper& system,
                                                                  const ContinuousTrajectory& linearizationTrajectory) {
  auto& t = linearizationTrajectory.timeTrajectory;
  auto& x = linearizationTrajectory.stateTrajectory;
  auto& u = linearizationTrajectory.inputTrajectory;
  const int N = t.size() - 1;

  // LinearQuadraticProblem with N+1 elements. Terminal stage lqp[N].dynamics is ignored.
  std::vector<LinearQuadraticStage> lqp;
  lqp.reserve(N + 1);
  for (int k = 0; k < N; ++k) {  // Intermediate stages
    lqp.emplace_back(discretizeStage(cost, system, {t[k], x[k], u[k]}, {t[k + 1], x[k + 1]}));
  }
  lqp.emplace_back(cost.getTerminalQuadraticApproximation(t[N], x[N]), LinearDynamics());  // Terminal cost, no dynamics.

  return lqp;
}

std::pair<std::vector<int>, std::vector<int>> getNumStatesAndInputs(const std::vector<LinearQuadraticStage>& linearQuadraticApproximation) {
  const int N = linearQuadraticApproximation.size() - 1;
  std::vector<int> numStates;
  std::vector<int> numInputs;
  numStates.reserve(N+1);
  numInputs.reserve(N);

  for (int k = 0; k < N; ++k) {
    numStates.push_back(linearQuadraticApproximation[k].dynamics.dfdx.cols());
    numInputs.push_back(linearQuadraticApproximation[k].dynamics.dfdu.cols());
  }
  numStates.push_back(linearQuadraticApproximation[N - 1].dynamics.dfdx.rows());

  return {numStates, numInputs};
}

LinearQuadraticStage discretizeStage(CostWrapper& cost, SystemWrapper& system, TrajectoryRef start, StateTrajectoryRef end) {
  LinearQuadraticStage lqStage;
  auto dt = end.t - start.t;

  lqStage.cost = discretizeCost(cost, start, dt);

  // Linearized Dynamics after discretization: x0[k+1] + dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k])
  lqStage.dynamics = discretizeDynamics(system, start, dt);
  // Adapt the offset to account for the defect along the linearization: dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k]) - x0[k+1]
  lqStage.dynamics.b -= end.x;

  return lqStage;
}

QuadraticCost discretizeCost(CostWrapper& cost, TrajectoryRef start, double dt) {
  // Approximates the cost accumulation of the dt interval.
  // Use Euler integration
  const auto continuousCosts = cost.getQuadraticApproximation(start.t, start.x, start.u);
  QuadraticCost discreteCosts;
  discreteCosts.Q = continuousCosts.Q * dt;
  discreteCosts.P = continuousCosts.P * dt;
  discreteCosts.R = continuousCosts.R * dt;
  discreteCosts.q = continuousCosts.q * dt;
  discreteCosts.r = continuousCosts.r * dt;
  discreteCosts.c = continuousCosts.c * dt;
  return discreteCosts;
}

LinearDynamics discretizeDynamics(SystemWrapper& system, TrajectoryRef start, double dt) {
  // Forward Euler discretization
  // x[k+1] = x[k] + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * (A_c dx[k] + B_c du[k] + b_c)
  // x[k+1] = (I + A_c * dt) dx[k] + (B_c * dt) du[k] + (b_c * dt + x0[k])
  const auto continuousDynamics = system.getLinearApproximation(start.t, start.x, start.u);
  LinearDynamics discreteDynamics;
  discreteDynamics.A = continuousDynamics.A * dt;
  discreteDynamics.A.diagonal().array() += 1.0;
  discreteDynamics.B = continuousDynamics.B * dt;
  discreteDynamics.b = continuousDynamics.b * dt + start.x;
  return discreteDynamics;
}

}  // namespace qp_solver
}  // namespace ocs2
