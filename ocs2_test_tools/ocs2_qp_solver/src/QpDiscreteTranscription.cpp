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
  lqp.emplace_back(cost.getTerminalQuadraticApproximation(t[N], x[N]), VectorFunctionLinearApproximation());  // Terminal cost, no dynamics.

  return lqp;
}

LinearQuadraticStage discretizeStage(CostWrapper& cost, SystemWrapper& system, TrajectoryRef start, StateTrajectoryRef end) {
  LinearQuadraticStage lqStage;
  auto dt = end.t - start.t;

  lqStage.cost = discretizeCost(cost, start, dt);

  // Linearized Dynamics after discretization: x0[k+1] + dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k])
  lqStage.dynamics = discretizeDynamics(system, start, dt);
  // Adapt the offset to account for the defect along the linearization: dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k]) - x0[k+1]
  lqStage.dynamics.f -= end.x;

  return lqStage;
}

ScalarFunctionQuadraticApproximation discretizeCost(CostWrapper& cost, TrajectoryRef start, double dt) {
  // Approximates the cost accumulation of the dt interval.
  // Use Euler integration
  const auto continuousCosts = cost.getQuadraticApproximation(start.t, start.x, start.u);
  ScalarFunctionQuadraticApproximation discreteCosts;
  discreteCosts.dfdxx = continuousCosts.dfdxx * dt;
  discreteCosts.dfdux = continuousCosts.dfdux * dt;
  discreteCosts.dfduu = continuousCosts.dfduu * dt;
  discreteCosts.dfdx = continuousCosts.dfdx * dt;
  discreteCosts.dfdu = continuousCosts.dfdu * dt;
  discreteCosts.f = continuousCosts.f * dt;
  return discreteCosts;
}

VectorFunctionLinearApproximation discretizeDynamics(SystemWrapper& system, TrajectoryRef start, double dt) {
  // Forward Euler discretization
  // x[k+1] = x[k] + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * (A_c dx[k] + B_c du[k] + b_c)
  // x[k+1] = (I + A_c * dt) dx[k] + (B_c * dt) du[k] + (b_c * dt + x0[k])
  const auto continuousDynamics = system.getLinearApproximation(start.t, start.x, start.u);
  VectorFunctionLinearApproximation discreteDynamics;
  discreteDynamics.dfdx = continuousDynamics.dfdx * dt;
  discreteDynamics.dfdx.diagonal().array() += 1.0;
  discreteDynamics.dfdu = continuousDynamics.dfdu * dt;
  discreteDynamics.f = continuousDynamics.f * dt + start.x;
  return discreteDynamics;
}

}  // namespace qp_solver
}  // namespace ocs2
