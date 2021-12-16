//
// Created by rgrandia on 21.01.21.
//

#include "ocs2_switched_model_interface/foot_planner/QuinticSplineSwing.h"

#include <ocs2_core/misc/Lookup.h>

#include <Eigen/Sparse>

namespace switched_model {

scalar_t QuinticSpline::position(scalar_t time) const {
  const scalar_t tau1 = normalizedTime(time);
  const scalar_t tau2 = tau1 * tau1;
  const scalar_t tau3 = tau2 * tau1;
  const scalar_t tau4 = tau3 * tau1;
  const scalar_t tau5 = tau4 * tau1;
  return c_[0] * tau5 + c_[1] * tau4 + c_[2] * tau3 + c_[3] * tau2 + c_[4] * tau1 + c_[5];
}

scalar_t QuinticSpline::velocity(scalar_t time) const {
  const scalar_t tau1 = normalizedTime(time);
  const scalar_t tau2 = tau1 * tau1;
  const scalar_t tau3 = tau2 * tau1;
  const scalar_t tau4 = tau3 * tau1;
  return (c_[0] * 5.0 * tau4 + c_[1] * 4.0 * tau3 + c_[2] * 3.0 * tau2 + c_[3] * 2.0 * tau1 + c_[4]) / dt_;
}

scalar_t QuinticSpline::acceleration(scalar_t time) const {
  const scalar_t tau1 = normalizedTime(time);
  const scalar_t tau2 = tau1 * tau1;
  const scalar_t tau3 = tau2 * tau1;
  return (c_[0] * 20.0 * tau3 + c_[1] * 12.0 * tau2 + c_[2] * 6.0 * tau1 + c_[3] * 2.0) / (dt_ * dt_);
}

scalar_t QuinticSpline::jerk(scalar_t time) const {
  const scalar_t tau1 = normalizedTime(time);
  const scalar_t tau2 = tau1 * tau1;
  return (c_[0] * 60.0 * tau2 + c_[1] * 24.0 * tau1 + c_[2] * 6.0) / (dt_ * dt_ * dt_);
}

scalar_t QuinticSpline::normalizedTime(scalar_t t) const {
  return (t - t0_) / dt_;
}

// helper functions
namespace {
void addStartPositionConstraint(scalar_t p, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b, int constraintIdx,
                                int splineIdx) {
  A.insert(constraintIdx, splineIdx + 5) = 1.0;
  b(constraintIdx) = p;
}

void addEndPositionConstraint(scalar_t p, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b, int constraintIdx,
                              int splineIdx) {
  A.insert(constraintIdx, splineIdx + 0) = 1.0;
  A.insert(constraintIdx, splineIdx + 1) = 1.0;
  A.insert(constraintIdx, splineIdx + 2) = 1.0;
  A.insert(constraintIdx, splineIdx + 3) = 1.0;
  A.insert(constraintIdx, splineIdx + 4) = 1.0;
  A.insert(constraintIdx, splineIdx + 5) = 1.0;
  b(constraintIdx) = p;
}

void addStartVelocityConstraint(scalar_t v, scalar_t dt, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b, int constraintIdx,
                                int splineIdx) {
  A.insert(constraintIdx, splineIdx + 4) = 1.0;
  b(constraintIdx) = v * dt;
}

void addEndVelocityConstraint(scalar_t v, scalar_t dt, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b, int constraintIdx,
                              int splineIdx) {
  A.insert(constraintIdx, splineIdx + 0) = 5.0;
  A.insert(constraintIdx, splineIdx + 1) = 4.0;
  A.insert(constraintIdx, splineIdx + 2) = 3.0;
  A.insert(constraintIdx, splineIdx + 3) = 2.0;
  A.insert(constraintIdx, splineIdx + 4) = 1.0;
  b(constraintIdx) = v * dt;
}

void addStartAccelerationConstraint(scalar_t a, scalar_t dt, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b,
                                    int constraintIdx, int splineIdx) {
  A.insert(constraintIdx, splineIdx + 3) = 2.0;
  b(constraintIdx) = a * dt * dt;
}

void addEndAccelerationConstraint(scalar_t a, scalar_t dt, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b,
                                  int constraintIdx, int splineIdx) {
  A.insert(constraintIdx, splineIdx + 0) = 20.0;
  A.insert(constraintIdx, splineIdx + 1) = 12.0;
  A.insert(constraintIdx, splineIdx + 2) = 6.0;
  A.insert(constraintIdx, splineIdx + 3) = 2.0;
  b(constraintIdx) = a * dt * dt;
}

void addAccelerationContinuity(scalar_t dt_lhs, scalar_t dt_rhs, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b,
                               int constraintIdx, int splineIdx_lhs, int splineIdx_rhs) {
  const scalar_t scaling = dt_lhs / dt_rhs;
  const scalar_t scaling2 = scaling * scaling;

  A.insert(constraintIdx, splineIdx_lhs + 0) = 20.0;
  A.insert(constraintIdx, splineIdx_lhs + 1) = 12.0;
  A.insert(constraintIdx, splineIdx_lhs + 2) = 6.0;
  A.insert(constraintIdx, splineIdx_lhs + 3) = 2.0;
  A.insert(constraintIdx, splineIdx_rhs + 3) = -2.0 * scaling2;
  b(constraintIdx) = 0.0;
}

void addJerkContinuity(scalar_t dt_lhs, scalar_t dt_rhs, Eigen::SparseMatrix<scalar_t, Eigen::RowMajor>& A, vector_t& b, int constraintIdx,
                       int splineIdx_lhs, int splineIdx_rhs) {
  const scalar_t scaling = dt_lhs / dt_rhs;
  const scalar_t scaling2 = scaling * scaling;
  const scalar_t scaling3 = scaling2 * scaling;

  A.insert(constraintIdx, splineIdx_lhs + 0) = 60.0;
  A.insert(constraintIdx, splineIdx_lhs + 1) = 24.0;
  A.insert(constraintIdx, splineIdx_lhs + 2) = 6.0;
  A.insert(constraintIdx, splineIdx_rhs + 2) = -6.0 * scaling3;
  b(constraintIdx) = 0.0;
}

}  // namespace

QuinticSwing::QuinticSwing() : splines({QuinticSpline()}) {}

QuinticSwing::QuinticSwing(const SwingNode& start, scalar_t midHeight, const SwingNode& end)
    : QuinticSwing(start, SwingNode{0.5 * (start.time + end.time), midHeight, 0.0}, end) {}

QuinticSwing::QuinticSwing(const SwingNode& start, const SwingNode& mid, const SwingNode& end)
    : QuinticSwing(std::vector<SwingNode>{start, mid, end}) {}

QuinticSwing::QuinticSwing(const std::vector<SwingNode>& nodes) {
  if (nodes.size() < 3) {
    throw std::runtime_error("[QuinticSwing] need at least 3 nodes for spline fitting");
  }

  // Set up normalized spline conditions as a linear system
  const size_t numSplines = nodes.size() - 1;
  const size_t numVariablesPerSpline = 6;
  const size_t numConditions = numVariablesPerSpline * numSplines;

  // Initialize sparse linear system A*x = b
  Eigen::SparseMatrix<scalar_t, Eigen::RowMajor> A(numConditions, numConditions);
  A.reserve(Eigen::VectorXi::Constant(numConditions, 6));  // Each constraint has max 6 entries
  vector_t b(numConditions);

  int constraintIdx = 0;
  for (int splineNum = 0; splineNum < numSplines; ++splineNum) {
    const int splineIdx = splineNum * numVariablesPerSpline;
    const scalar_t dt = nodes[splineNum + 1].time - nodes[splineNum].time;

    // check sorting
    assert(dt > 0.0);

    // Position is given
    addStartPositionConstraint(nodes[splineNum].position, A, b, constraintIdx++, splineIdx);
    addEndPositionConstraint(nodes[splineNum + 1].position, A, b, constraintIdx++, splineIdx);

    // Velocity is given
    addStartVelocityConstraint(nodes[splineNum].velocity, dt, A, b, constraintIdx++, splineIdx);
    addEndVelocityConstraint(nodes[splineNum + 1].velocity, dt, A, b, constraintIdx++, splineIdx);

    // Acceleration is 0.0 at start and end of the curve
    if (splineNum == 0) {
      addStartAccelerationConstraint(0.0, dt, A, b, constraintIdx++, splineIdx);
    }
    if (splineNum == (numSplines - 1)) {
      addEndAccelerationConstraint(0.0, dt, A, b, constraintIdx++, splineIdx);
    }

    // Acceleration and jerk continuity for middle nodes
    if (splineNum != (numSplines - 1)) {
      const scalar_t dt_next = nodes[splineNum + 2].time - nodes[splineNum + 1].time;
      addAccelerationContinuity(dt, dt_next, A, b, constraintIdx++, splineIdx, splineIdx + numVariablesPerSpline);
      addJerkContinuity(dt, dt_next, A, b, constraintIdx++, splineIdx, splineIdx + numVariablesPerSpline);
    }
  }
  // Expected number of constraints was added
  assert(constraintIdx == numConditions);

  // Solve with x = inv(A'*A) * A' * b
  Eigen::SparseMatrix<scalar_t> AtA = A.adjoint() * A;
  Eigen::SimplicialCholesky<Eigen::SparseMatrix<scalar_t>> chol(AtA);
  vector_t rhs = A.adjoint() * b;
  vector_t coefficients = chol.solve(rhs);

  assert(coefficients.allFinite());

  // Extract and store results
  for (int splineNum = 0; splineNum < numSplines; ++splineNum) {
    const int splineIdx = splineNum * numVariablesPerSpline;
    const scalar_t dt = nodes[splineNum + 1].time - nodes[splineNum].time;
    splines.push_back(QuinticSpline(coefficients.segment<numVariablesPerSpline>(splineIdx), nodes[splineNum].time, dt));

    // add when this spline ends (except for the last one)
    if (splineNum != (numSplines - 1)) {
      nodeTimes.push_back(nodes[splineNum + 1].time);
    }
  }
}

scalar_t QuinticSwing::position(scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(nodeTimes, time);
  return splines[index].position(time);
}

scalar_t QuinticSwing::velocity(scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(nodeTimes, time);
  return splines[index].velocity(time);
}

scalar_t QuinticSwing::acceleration(scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(nodeTimes, time);
  return splines[index].acceleration(time);
}

scalar_t QuinticSwing::jerk(scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(nodeTimes, time);
  return splines[index].jerk(time);
}

}  // namespace switched_model