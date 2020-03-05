//
// Created by rgrandia on 28.02.20.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ocs2 {
namespace qp_solver {

/** A time, state, input trajectory. The last timepoint has only a state, no input */
struct ContinuousTrajectory {
  /** time trajectory, size N+1 */  // NOLINTNEXTLINE
  std::vector<double> timeTrajectory;
  /** trajectory of state vectors, size N+1 */  // NOLINTNEXTLINE
  std::vector<Eigen::VectorXd> stateTrajectory;
  /** trajectory of input vectors, size N */  // NOLINTNEXTLINE
  std::vector<Eigen::VectorXd> inputTrajectory;
};

/** Reference to a point along a trajectory. Does not own the state-input data. */
struct TrajectoryRef {
  /** time */  // NOLINTNEXTLINE
  double t;
  /** state */  // NOLINTNEXTLINE
  const Eigen::VectorXd& x;
  /** input */  // NOLINTNEXTLINE
  const Eigen::VectorXd& u;
};

/** Reference to the state at a point along a trajectory. Does not own the state data. */
struct StateTrajectoryRef {
  /** time */  // NOLINTNEXTLINE
  double t;
  /** state */  // NOLINTNEXTLINE
  const Eigen::VectorXd& x;
};

/** Adds state and inputs of two trajectories, time is not added. */
ContinuousTrajectory add(const ContinuousTrajectory& t0, const ContinuousTrajectory& t1);

}  // namespace qp_solver
}  // namespace ocs2
