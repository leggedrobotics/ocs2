/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <gtest/gtest.h>
#include <iostream>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearAlgebra.h>

#include "ocs2_legged_robot/constraint/FrictionConeConstraint.h"
#include "ocs2_legged_robot/test/AnymalFactoryFunctions.h"

using namespace ocs2;
using namespace legged_robot;

class TestFrictionConeConstraint : public testing::Test {
 public:
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  TestFrictionConeConstraint() {}

  const CentroidalModelType centroidalModelType = CentroidalModelType::SingleRigidBodyDynamics;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr = createAnymalPinocchioInterface();
  const CentroidalModelInfo centroidalModelInfo = createAnymalCentroidalModelInfo(*pinocchioInterfacePtr, centroidalModelType);
  const std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr =
      createReferenceManager(centroidalModelInfo.numThreeDofContacts);
  PreComputation preComputation;
};

TEST_F(TestFrictionConeConstraint, finiteDifference) {
  const FrictionConeConstraint::Config config;

  scalar_t t = 0.0;
  scalar_t eps = 1e-4;
  scalar_t tol = 1e-2;  // tolerance on the Jacobian elements
  size_t N = 10000;

  for (size_t legNumber = 0; legNumber < centroidalModelInfo.numThreeDofContacts; ++legNumber) {
    FrictionConeConstraint frictionConeConstraint(*referenceManagerPtr, config, legNumber, centroidalModelInfo);

    vector_t u0 = 10.0 * vector_t::Random(centroidalModelInfo.inputDim);
    u0(2) = 100.0;
    u0(5) = 100.0;
    u0(8) = 100.0;
    u0(11) = 100.0;
    vector_t x0 = 0.1 * vector_t::Random(centroidalModelInfo.stateDim);
    const auto y0 = frictionConeConstraint.getValue(t, x0, u0, preComputation)(0);
    auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x0, u0, preComputation);

    vector_t data(N);
    matrix_t regressor(N, 6 + 6 + 6 + 9);
    vector_t dx = vector_t::Zero(centroidalModelInfo.stateDim);
    vector_t du = vector_t::Zero(centroidalModelInfo.inputDim);
    for (size_t i = 0; i < N; i++) {
      // evaluation point
      vector_t dEuler = eps * vector_t::Random(3);
      vector_t dF = eps * vector_t::Random(3);
      dx.segment<3>(0) = dEuler;
      du.segment<3>(3 * legNumber) = dF;
      vector_t dz(6);
      dz << dEuler, dF;
      const matrix_t quadTerms = dz * dz.transpose();
      vector_t quadTermsVector(6 + 6 + 9);
      size_t count = 0;
      for (size_t p = 0; p < 6; ++p) {
        for (size_t q = p; q < 6; ++q) {
          quadTermsVector(count) = quadTerms(p, q);
          if (q == p) {
            quadTermsVector(count) *= 0.5;
          }
          count++;
        }
      }

      // Scale to condition the regressor
      regressor.row(i) << dEuler.transpose() / eps, dF.transpose() / eps, quadTermsVector.transpose() / (eps * eps);
      data(i) = (frictionConeConstraint.getValue(t, x0 + dx, u0 + du, preComputation)(0) - y0);
    }

    vector_t dh_emperical = regressor.colPivHouseholderQr().solve(data);
    dh_emperical /= eps;
    dh_emperical.tail<6 + 6 + 9>() /= eps;

    matrix_t quadTerms(6, 6);
    size_t count = 0;
    for (size_t p = 0; p < 6; ++p) {
      for (size_t q = p; q < 6; ++q) {
        quadTerms(p, q) = dh_emperical(6 + count);
        quadTerms(q, p) = quadTerms(p, q);
        count++;
      }
    }

    vector_t dhdx_emperical = dh_emperical.head<3>();
    vector_t dhdu_emperical = dh_emperical.segment<3>(3);
    matrix_t ddhdxdx_emperical = quadTerms.block<3, 3>(0, 0);
    matrix_t ddhdudx_emperical = quadTerms.block<3, 3>(3, 0);
    matrix_t ddhdudu_emperical = quadTerms.block<3, 3>(3, 3);

    matrix_t ddhdudu = quadraticApproximation.dfduu.front().block<3, 3>(3 * legNumber, 3 * legNumber);
    matrix_t ddhdxdx = quadraticApproximation.dfdxx.front().block<3, 3>(0, 0);
    ASSERT_LT((dhdx_emperical - quadraticApproximation.dfdx.block<1, 3>(0, 0).transpose()).array().abs().maxCoeff(), tol);
    ASSERT_LT((dhdu_emperical - quadraticApproximation.dfdu.block<1, 3>(0, 3 * legNumber).transpose()).array().abs().maxCoeff(), tol);
    ASSERT_LT((ddhdudu_emperical - ddhdudu).array().abs().maxCoeff(), tol);
    // ddhdxdx and ddhdudx are off because of the negative definite hessian approximation
  }
}

TEST_F(TestFrictionConeConstraint, gravityAligned_flatTerrain) {
  // Check friction cone for the case where the body is aligned with the terrain
  const FrictionConeConstraint::Config config(0.7, 25.0, 0.0, 0.0);
  const auto mu = config.frictionCoefficient;
  const auto regularization = config.regularization;

  // evaluation point
  scalar_t t = 0.0;
  vector_t x = vector_t::Random(centroidalModelInfo.stateDim);
  vector_t u = vector_t::Random(centroidalModelInfo.inputDim);

  for (size_t legNumber = 0; legNumber < centroidalModelInfo.numThreeDofContacts; ++legNumber) {
    FrictionConeConstraint frictionConeConstraint(*referenceManagerPtr, config, legNumber, centroidalModelInfo);

    // Local forces are equal to the body forces.
    const vector_t F = centroidal_model::getContactForces(u, legNumber, centroidalModelInfo);
    const auto Fx = F(0);
    const auto Fy = F(1);
    const auto Fz = F(2);

    auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x, u, preComputation);

    ASSERT_DOUBLE_EQ(quadraticApproximation.f(0), Fz * sqrt(mu * mu) - sqrt(Fx * Fx + Fy * Fy + regularization));

    // First derivative inputs
    vector_t dhdu = vector_t::Zero(centroidalModelInfo.inputDim);
    const auto F_norm = sqrt(Fx * Fx + Fy * Fy + regularization);
    dhdu(3 * legNumber + 0) = -Fx / F_norm;
    dhdu(3 * legNumber + 1) = -Fy / F_norm;
    dhdu(3 * legNumber + 2) = sqrt(mu * mu);

    ASSERT_LT((quadraticApproximation.dfdu.row(0).transpose() - dhdu).norm(), 1e-12);

    // Second derivative inputs
    matrix_t ddhdudu = matrix_t::Zero(centroidalModelInfo.inputDim, centroidalModelInfo.inputDim);
    const auto F_norm2 = Fx * Fx + Fy * Fy + regularization;
    const auto F_norm32 = pow(F_norm2, 1.5);
    ddhdudu(3 * legNumber + 0, 3 * legNumber + 0) = -(Fy * Fy + regularization) / F_norm32;
    ddhdudu(3 * legNumber + 0, 3 * legNumber + 1) = Fx * Fy / F_norm32;
    ddhdudu(3 * legNumber + 0, 3 * legNumber + 2) = 0.0;
    ddhdudu(3 * legNumber + 1, 3 * legNumber + 0) = Fx * Fy / F_norm32;
    ddhdudu(3 * legNumber + 1, 3 * legNumber + 1) = -(Fx * Fx + regularization) / F_norm32;
    ddhdudu(3 * legNumber + 1, 3 * legNumber + 2) = 0.0;
    ddhdudu(3 * legNumber + 2, 3 * legNumber + 0) = 0.0;
    ddhdudu(3 * legNumber + 2, 3 * legNumber + 1) = 0.0;
    ddhdudu(3 * legNumber + 2, 3 * legNumber + 2) = 0.0;

    ASSERT_LT((quadraticApproximation.dfduu.front() - ddhdudu).norm(), 1e-12);
  }
}

TEST_F(TestFrictionConeConstraint, negativeDefinite) {
  const FrictionConeConstraint::Config config;

  // evaluation point
  scalar_t t = 0.0;
  vector_t x = vector_t::Random(centroidalModelInfo.stateDim);
  vector_t u = vector_t::Random(centroidalModelInfo.inputDim);
  u(2) = 100.0;
  u(5) = 100.0;
  u(8) = 100.0;
  u(11) = 100.0;

  for (size_t legNumber = 0; legNumber < centroidalModelInfo.numThreeDofContacts; ++legNumber) {
    FrictionConeConstraint frictionConeConstraint(*referenceManagerPtr, config, legNumber, centroidalModelInfo);

    const auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x, u, preComputation);
    ASSERT_LT(LinearAlgebra::symmetricEigenvalues(quadraticApproximation.dfdxx.front()).maxCoeff(), 0.0);
    ASSERT_LT(LinearAlgebra::symmetricEigenvalues(quadraticApproximation.dfduu.front()).maxCoeff(), 0.0);
  }
}
