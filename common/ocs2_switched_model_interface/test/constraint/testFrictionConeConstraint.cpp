//
// Created by rgrandia on 19.09.19.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

TEST(TestFrictionConeConstraint, finiteDifference) {
  using TestedConstraint = switched_model::FrictionConeConstraint;
  const double mu = 0.7;
  const double regularization = 25;
  TestedConstraint frictionConeConstraint(mu, regularization, 0);

  double t = 0.0;
  double eps = 1e-4;
  double tol = 1e-2;  // tolerance on the Jacobian elements
  int N = 10000;

  for (int legNumber = 0; legNumber < switched_model::NUM_CONTACT_POINTS; ++legNumber) {
    TestedConstraint frictionConeConstraint(mu, regularization, legNumber);
    switched_model::vector3_t surfaceNormal = switched_model::vector3_t{0.0, 0.0, 1.0} + 0.1*switched_model::vector3_t::Random();
    surfaceNormal.normalize();
    frictionConeConstraint.setSurfaceNormalInWorld(surfaceNormal);

    TestedConstraint::input_vector_t u0 = 10.0 * TestedConstraint::input_vector_t::Random();
    u0(2) = 100.0;
    u0(5) = 100.0;
    u0(8) = 100.0;
    u0(11) = 100.0;
    TestedConstraint::state_vector_t x0 = 0.1 * TestedConstraint::input_vector_t::Random();
    const auto y0 = frictionConeConstraint.getValue(t, x0, u0).front();
    auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x0, u0);

    switched_model::dynamic_vector_t data(N);
    switched_model::dynamic_matrix_t regressor(N, 6+ 6+6+9);
    TestedConstraint::state_vector_t dx = TestedConstraint::state_vector_t::Zero();
    TestedConstraint::input_vector_t du = TestedConstraint::input_vector_t::Zero();
    for (int i = 0; i < N; i++) {
      // evaluation point
      switched_model::vector3_t dEuler = eps * switched_model::vector3_t::Random();
      switched_model::vector3_t dF = eps * switched_model::vector3_t::Random();
      dx.segment<3>(0) = dEuler;
      du.segment<3>(3*legNumber) = dF;
      switched_model::vector6_t dz;
      dz << dEuler, dF;
      const switched_model::dynamic_matrix_t quadTerms = dz * dz.transpose();
      switched_model::dynamic_vector_t quadTermsVector(6+6+9);
      int count =0;
      for (int p=0; p<6; ++p) {
        for (int q=p; q<6; ++q) {
          quadTermsVector(count) = quadTerms(p, q);
          if (q==p) {
            quadTermsVector(count) *= 0.5;
          }
          count++;
        }
      }

      // Scale to condition the regressor
      regressor.row(i) << dEuler.transpose() / eps, dF.transpose() / eps, quadTermsVector.transpose() / (eps * eps);
      data(i)  = (frictionConeConstraint.getValue(t, x0 + dx, u0 + du).front() - y0);
    }

    switched_model::dynamic_vector_t dh_emperical = regressor.colPivHouseholderQr().solve(data);
    dh_emperical /= eps;
    dh_emperical.tail<6+6+9>()  /= eps;

    switched_model::dynamic_matrix_t quadTerms(6, 6);
    int count = 0;
    for (int p=0; p<6; ++p) {
      for (int q=p; q<6; ++q) {
        quadTerms(p, q) = dh_emperical(6+count);
        quadTerms(q, p) = quadTerms(p, q);
        count++;
      }
    }

    switched_model::vector3_t dhdx_emperical = dh_emperical.head<3>();
    switched_model::vector3_t dhdu_emperical = dh_emperical.segment<3>(3);
    switched_model::matrix3_t ddhdxdx_emperical = quadTerms.block<3, 3>(0, 0);
    switched_model::matrix3_t ddhdudx_emperical = quadTerms.block<3, 3>(3, 0);
    switched_model::matrix3_t ddhdudu_emperical = quadTerms.block<3, 3>(3, 3);

    switched_model::matrix3_t ddhdudu = quadraticApproximation.secondDerivativesInput.front().block<3, 3>(3*legNumber, 3*legNumber);
    switched_model::matrix3_t ddhdxdx = quadraticApproximation.secondDerivativesState.front().block<3, 3>(0, 0);
    ASSERT_LT((dhdx_emperical - quadraticApproximation.derivativeState.front().segment<3>(0) ).array().abs().maxCoeff(), tol);
    ASSERT_LT((dhdu_emperical - quadraticApproximation.derivativeInput.front().segment<3>(3*legNumber) ).array().abs().maxCoeff(), tol);
    ASSERT_LT((ddhdudu_emperical - ddhdudu).array().abs().maxCoeff(), tol);
    // ddhdxdx and ddhdudx are off because of the negative definite hessian approximation
  }
}

TEST(TestFrictionConeConstraint, gravityAligned_flatTerrain) {
  // Check friction cone for the case where the body is aligned with the terrain

  using TestedConstraint = switched_model::FrictionConeConstraint;
  const double mu = 0.7;
  const double regularization = 25;

  // evaluation point
  double t = 0.0;
  TestedConstraint::input_vector_t u;
  TestedConstraint::state_vector_t x;
  u.setRandom();
  x.setRandom();

  const switched_model::vector3_t eulerXYZ = switched_model::getOrientation(switched_model::getComPose(x));
  switched_model::TerrainPlane terrainPlane;
  terrainPlane.orientationWorldToTerrain = switched_model::rotationMatrixBaseToOrigin(eulerXYZ).transpose();

  for (int legNumber = 0; legNumber < switched_model::NUM_CONTACT_POINTS; ++legNumber) {
    TestedConstraint frictionConeConstraint(mu, regularization, legNumber);
    frictionConeConstraint.setSurfaceNormalInWorld(switched_model::surfaceNormalInWorld(terrainPlane));

    // Local forces are equal to the body forces.
    const auto Fx = u(3 * legNumber + 0);
    const auto Fy = u(3 * legNumber + 1);
    const auto Fz = u(3 * legNumber + 2);

    auto quadraticApproximation = frictionConeConstraint.getQuadraticApproximation(t, x, u);

    ASSERT_DOUBLE_EQ(quadraticApproximation.constraintValues.front(), Fz * sqrt(mu * mu) - sqrt(Fx * Fx + Fy * Fy + regularization));

    // First derivative inputs
    switched_model::FrictionConeConstraint::input_vector_t dhdu;
    dhdu.setZero();
    const auto F_norm = sqrt(Fx * Fx + Fy * Fy + regularization);
    dhdu(3 * legNumber + 0) = -Fx / F_norm;
    dhdu(3 * legNumber + 1) = -Fy / F_norm;
    dhdu(3 * legNumber + 2) = sqrt(mu * mu);

    ASSERT_LT((quadraticApproximation.derivativeInput.front() - dhdu).norm(), 1e-12);

    // Second derivative inputs
    switched_model::FrictionConeConstraint::input_matrix_t ddhdudu;
    ddhdudu.setZero();
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

    ASSERT_LT((quadraticApproximation.secondDerivativesInput.front() - ddhdudu).norm(), 1e-12);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
