//
// Created by rgrandia on 28.02.20.
//

#include "ocs2_qp_solver/QpTrajectories.h"

namespace ocs2 {
namespace qp_solver {

ContinuousTrajectory add(const ContinuousTrajectory& t0, const ContinuousTrajectory& t1) {
  const int N = t0.timeTrajectory.size() - 1;
  ContinuousTrajectory sum;
  sum.timeTrajectory = t0.timeTrajectory;
  sum.inputTrajectory.reserve(N);
  sum.stateTrajectory.reserve(N + 1);

  for (int k = 0; k < N; ++k) {
    sum.inputTrajectory.emplace_back(t0.inputTrajectory[k] + t1.inputTrajectory[k]);
    sum.stateTrajectory.emplace_back(t0.stateTrajectory[k] + t1.stateTrajectory[k]);
  }
  sum.stateTrajectory.emplace_back(t0.stateTrajectory[N] + t1.stateTrajectory[N]);
  return sum;
}

}  // namespace qp_solver
}  // namespace ocs2
