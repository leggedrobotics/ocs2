//
// Created by rgrandia on 19.09.19.
//

#include <gtest/gtest.h>

#include <ocs2_core/misc/LinearAlgebra.h>

#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"
#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

using namespace switched_model;

TEST(TestFrictionConeConstraint, finiteDifference) {
  // Mock the modeScheduleManager
  SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
  modeScheduleManager.setModeSchedule({{}, {ModeNumber::STANCE}});

  // Mock the precomputation
  SwitchedModelPreComputationMockup preComp;

  FrictionConeConstraint::Config config;

  scalar_t t = 0.0;
  scalar_t eps = 1e-4;
  scalar_t tol = 1e-2;  // tolerance on the Jacobian elements
  int N = 10000;

  for (int legNumber = 0; legNumber < NUM_CONTACT_POINTS; ++legNumber) {
    FrictionConeConstraint frictionConeConstraint(config, legNumber, modeScheduleManager);

    vector3_t surfaceNormal = vector3_t{0.0, 0.0, 1.0} + 0.1 * vector3_t::Random();
    surfaceNormal.normalize();

    preComp.surfaceNormalInOriginFrame(legNumber) = surfaceNormal;

    vector_t u0 = 10.0 * vector_t::Random(INPUT_DIM);
    u0(2) = 100.0;
    u0(5) = 100.0;
    u0(8) = 100.0;
    u0(11) = 100.0;
    vector_t x0 = 0.1 * vector_t::Random(STATE_DIM);
    const auto y0 = frictionConeConstraint.getValue(t, x0, u0, preComp)(0);
    auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x0, u0, preComp);

    vector_t data(N);
    matrix_t regressor(N, 6 + 6 + 6 + 9);
    vector_t dx = vector_t::Zero(STATE_DIM);
    vector_t du = vector_t::Zero(INPUT_DIM);
    for (int i = 0; i < N; i++) {
      // evaluation point
      vector3_t dEuler = eps * vector3_t::Random();
      vector3_t dF = eps * vector3_t::Random();
      dx.segment<3>(0) = dEuler;
      du.segment<3>(3 * legNumber) = dF;
      vector6_t dz;
      dz << dEuler, dF;
      const matrix_t quadTerms = dz * dz.transpose();
      vector_t quadTermsVector(6 + 6 + 9);
      int count = 0;
      for (int p = 0; p < 6; ++p) {
        for (int q = p; q < 6; ++q) {
          quadTermsVector(count) = quadTerms(p, q);
          if (q == p) {
            quadTermsVector(count) *= 0.5;
          }
          count++;
        }
      }

      // Scale to condition the regressor
      regressor.row(i) << dEuler.transpose() / eps, dF.transpose() / eps, quadTermsVector.transpose() / (eps * eps);
      data(i) = (frictionConeConstraint.getValue(t, x0 + dx, u0 + du, preComp)(0) - y0);
    }

    vector_t dh_emperical = regressor.colPivHouseholderQr().solve(data);
    dh_emperical /= eps;
    dh_emperical.tail<6 + 6 + 9>() /= eps;

    matrix_t quadTerms(6, 6);
    int count = 0;
    for (int p = 0; p < 6; ++p) {
      for (int q = p; q < 6; ++q) {
        quadTerms(p, q) = dh_emperical(6 + count);
        quadTerms(q, p) = quadTerms(p, q);
        count++;
      }
    }

    vector3_t dhdx_emperical = dh_emperical.head<3>();
    vector3_t dhdu_emperical = dh_emperical.segment<3>(3);
    matrix3_t ddhdxdx_emperical = quadTerms.block<3, 3>(0, 0);
    matrix3_t ddhdudx_emperical = quadTerms.block<3, 3>(3, 0);
    matrix3_t ddhdudu_emperical = quadTerms.block<3, 3>(3, 3);

    matrix3_t ddhdudu = quadraticApproximation.dfduu.front().block<3, 3>(3 * legNumber, 3 * legNumber);
    matrix3_t ddhdxdx = quadraticApproximation.dfdxx.front().block<3, 3>(0, 0);
    ASSERT_LT((dhdx_emperical - quadraticApproximation.dfdx.block<1, 3>(0, 0).transpose()).array().abs().maxCoeff(), tol);
    ASSERT_LT((dhdu_emperical - quadraticApproximation.dfdu.block<1, 3>(0, 3 * legNumber).transpose()).array().abs().maxCoeff(), tol);
    ASSERT_LT((ddhdudu_emperical - ddhdudu).array().abs().maxCoeff(), tol);
    // ddhdxdx and ddhdudx are off because of the negative definite hessian approximation
  }
}

TEST(TestFrictionConeConstraint, gravityAligned_flatTerrain) {
  // Mock the modeScheduleManager
  SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
  modeScheduleManager.setModeSchedule({{}, {ModeNumber::STANCE}});

  // Mock the precomputation
  SwitchedModelPreComputationMockup preComp;

  // Check friction cone for the case where the body is aligned with the terrain
  using TestedConstraint = FrictionConeConstraint;
  const FrictionConeConstraint::Config config(0.7, 25.0, 0.0, 0.0);
  const auto mu = config.frictionCoefficient;
  const auto regularization = config.regularization;

  // evaluation point
  scalar_t t = 0.0;
  comkino_state_t x = comkino_state_t::Random();
  comkino_input_t u = comkino_input_t::Random();

  // Set terrain parallel to the body
  const vector3_t eulerXYZ = getOrientation(getBasePose(x));
  TerrainPlane terrainPlane;
  terrainPlane.orientationWorldToTerrain = rotationMatrixOriginToBase(eulerXYZ);

  for (int legNumber = 0; legNumber < NUM_CONTACT_POINTS; ++legNumber) {
    TestedConstraint frictionConeConstraint(config, legNumber, modeScheduleManager);
    preComp.surfaceNormalInOriginFrame(legNumber) = surfaceNormalInWorld(terrainPlane);

    // Local forces are equal to the body forces.
    const auto Fx = u(3 * legNumber + 0);
    const auto Fy = u(3 * legNumber + 1);
    const auto Fz = u(3 * legNumber + 2);

    auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x, u, preComp);

    ASSERT_DOUBLE_EQ(quadraticApproximation.f(0), Fz * sqrt(mu * mu) - sqrt(Fx * Fx + Fy * Fy + regularization));

    // First derivative inputs
    vector_t dhdu = vector_t::Zero(INPUT_DIM);
    const auto F_norm = sqrt(Fx * Fx + Fy * Fy + regularization);
    dhdu(3 * legNumber + 0) = -Fx / F_norm;
    dhdu(3 * legNumber + 1) = -Fy / F_norm;
    dhdu(3 * legNumber + 2) = sqrt(mu * mu);

    ASSERT_LT((quadraticApproximation.dfdu.row(0).transpose() - dhdu).norm(), 1e-12);

    // Second derivative inputs
    matrix_t ddhdudu = matrix_t::Zero(INPUT_DIM, INPUT_DIM);
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

TEST(TestFrictionConeConstraint, negativeDefinite) {
  // Mock the modeScheduleManager
  SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
  modeScheduleManager.setModeSchedule({{}, {ModeNumber::STANCE}});

  // Mock the precomputation
  SwitchedModelPreComputationMockup preComp;

  using TestedConstraint = FrictionConeConstraint;
  const FrictionConeConstraint::Config config;

  // evaluation point
  scalar_t t = 0.0;
  comkino_state_t x = 0.1 * comkino_state_t::Random();
  comkino_input_t u = comkino_input_t::Random();
  u(2) = 100.0;
  u(5) = 100.0;
  u(8) = 100.0;
  u(11) = 100.0;

  // Flat terrain
  TerrainPlane terrainPlane;

  const scalar_t tol = 1e-12;
  for (int legNumber = 0; legNumber < NUM_CONTACT_POINTS; ++legNumber) {
    TestedConstraint frictionConeConstraint(config, legNumber, modeScheduleManager);
    preComp.surfaceNormalInOriginFrame(legNumber) = surfaceNormalInWorld(terrainPlane);

    const auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x, u, preComp);
    ASSERT_LT(ocs2::LinearAlgebra::symmetricEigenvalues(quadraticApproximation.dfdxx.front()).maxCoeff(), tol);
    ASSERT_LT(ocs2::LinearAlgebra::symmetricEigenvalues(quadraticApproximation.dfduu.front()).maxCoeff(), tol);
  }
}