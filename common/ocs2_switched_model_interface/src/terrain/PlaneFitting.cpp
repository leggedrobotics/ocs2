//
// Created by rgrandia on 27.11.20.
//

#include "ocs2_switched_model_interface/terrain/PlaneFitting.h"

namespace switched_model {

NormalAndPosition estimatePlane(const std::vector<vector3_t>& regressionPoints) {
  // Fit a plane that minimizes the normal distance to the provided regression points. Assumes at least 3 points are given.
  assert(regressionPoints.size() >= 3);

  // Collect statistics of the points
  size_t nPoints = 0;
  vector3_t sum = vector3_t::Zero();
  matrix3_t sumSquared = matrix3_t::Zero();
  for (const auto& point : regressionPoints) {
    nPoints++;
    sum += point;
    sumSquared.noalias() += point * point.transpose();
  }
  const vector3_t mean = sum / nPoints;
  const matrix3_t covarianceMatrix = sumSquared / nPoints - mean * mean.transpose();

  // Compute Eigenvectors.
  // Eigenvalues are ordered small to large.
  // Worst case bound for zero eigenvalue from : https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
  constexpr double eigenValueZeroTolerance = 1e-8;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(covarianceMatrix, Eigen::DecompositionOptions::ComputeEigenvectors);
  if (solver.eigenvalues()(1) > eigenValueZeroTolerance) {
    vector3_t unitaryNormalVector = solver.eigenvectors().col(0);

    // Check direction of the normal vector and flip the sign upwards
    if (unitaryNormalVector.z() < 0.0) {
      unitaryNormalVector = -unitaryNormalVector;
    }
    return {unitaryNormalVector, mean};
  } else {  // If second eigenvalue is zero, the normal is not defined. E.g. all points are on a line.
    return {vector3_t::UnitZ(), mean};
  }
}

}  // namespace switched_model