//
// Created by rgrandia on 28.02.20.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ocs2_qp_solver {

/** A time, state, input trajectory. The last timepoint has only a state, no input */
struct ContinuousTrajectory {
  std::vector<double> timeTrajectory;            // size N+1
  std::vector<Eigen::VectorXd> stateTrajectory;  // size N+1
  std::vector<Eigen::VectorXd> inputTrajectory;  // size N
};

/** Reference to a point along a trajectory. Does not own the state-input data. */
struct TrajectoryRef {
  double t_;
  const Eigen::VectorXd& x_;
  const Eigen::VectorXd& u_;
};

/** Reference to the state at a point along a trajectory. Does not own the state data. */
struct StateTrajectoryRef {
  double t_;
  const Eigen::VectorXd& x_;
};

/** Adds state and inputs of two trajectories, time is not added. */
ContinuousTrajectory add(const ContinuousTrajectory& t0, const ContinuousTrajectory& t1);

}  // namespace ocs2_qp_solver
