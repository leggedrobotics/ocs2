//
// Created by rgrandia on 19.09.19.
//

#include <gtest/gtest.h>

#include <ocs2_core/misc/LinearAlgebra.h>

#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/core/Rotations.h"

using namespace switched_model;

TEST(TestFrictionConeConstraint, finiteDifference) {
  friction_cone::Config config;
  config.hessianDiagonalShift = 0.0;  // no Hessian adaptation in this test

  const scalar_t eps = 1e-4;
  const scalar_t tol = 1e-9;
  const int N = 10000;

  for (int i = 0; i < N; ++i) {
    vector3_t forcesInTerrainFrame = 100.0 * vector3_t::Random();

    const auto y0 = frictionConeConstraint(config, forcesInTerrainFrame);
    auto quadraticApproximation = frictionConeLocalDerivatives(config, forcesInTerrainFrame);

    vector3_t dF = eps * vector3_t::Random();
    const auto y_eps = frictionConeConstraint(config, forcesInTerrainFrame + dF);
    const auto y_quadmodel = quadraticApproximation.coneConstraint + quadraticApproximation.dCone_dF.dot(dF) +
                             0.5 * dF.dot(quadraticApproximation.d2Cone_dF2 * dF);

    ASSERT_DOUBLE_EQ(y0, quadraticApproximation.coneConstraint);
    ASSERT_LT(std::abs(y_eps - y_quadmodel), tol);
  }
}

TEST(TestFrictionConeConstraint, negativeDefinite) {
  friction_cone::Config config;

  const scalar_t tol = 1e-12;
  const int N = 10000;

  for (int i = 0; i < N; ++i) {
    vector3_t forcesInTerrainFrame = 100.0 * vector3_t::Random();
    auto quadraticApproximation = frictionConeLocalDerivatives(config, forcesInTerrainFrame);
    ASSERT_LT(ocs2::LinearAlgebra::symmetricEigenvalues(quadraticApproximation.d2Cone_dF2).maxCoeff(), tol);
  }
}

TEST(TestFrictionConeConstraint, finiteDifference_body_frame) {
  friction_cone::Config config;
  config.hessianDiagonalShift = 0.0;  // no Hessian adaptation in this test

  const scalar_t eps = 1e-4;
  const scalar_t tol = 1e-5; // Tolerance not very tight because of the Gauss-Newton approximation in the body->terrain nonlinearity
  const int N = 10000;

  for (int i = 0; i < N; ++i) {
    // Random forces, orientation, and terrain direction
    const vector3_t forcesInBodyFrame = 100.0 * vector3_t::Random();
    const vector3_t eulerXYZ = vector3_t::Random();
    const matrix3_t t_R_w = rotationMatrixOriginToBase<scalar_t>(vector3_t::Random());

    // Compute additional inputs consistently
    const matrix3_t w_R_b = rotationMatrixBaseToOrigin(eulerXYZ);
    const vector3_t forcesInTerrainFrame = t_R_w * w_R_b * forcesInBodyFrame;

    const auto y0 = frictionConeConstraint(config, forcesInTerrainFrame);
    auto quadraticApproximation = frictionConeDerivatives(config, forcesInTerrainFrame, t_R_w, w_R_b, eulerXYZ, forcesInBodyFrame);

    // Perturb force in body and body orientation
    vector3_t dF = eps * vector3_t::Random();
    vector3_t dEuler = eps * vector3_t::Random();

    // Compute perturbed inputs consistently
    const matrix3_t w_R_b_eps = rotationMatrixBaseToOrigin<scalar_t>(eulerXYZ + dEuler);
    const vector3_t forcesInTerrainFrame_eps = t_R_w * w_R_b_eps * (forcesInBodyFrame + dF);

    const auto y_eps = frictionConeConstraint(config, forcesInTerrainFrame_eps);
    const auto y_quadmodel = quadraticApproximation.coneConstraint + quadraticApproximation.dCone_deuler.dot(dEuler) +
                             quadraticApproximation.dCone_du.dot(dF) + 0.5 * dEuler.dot(quadraticApproximation.d2Cone_deuler2 * dEuler) +
                             dF.dot(quadraticApproximation.d2Cone_dudeuler * dEuler) + 0.5 * dF.dot(quadraticApproximation.d2Cone_du2 * dF);

    ASSERT_DOUBLE_EQ(y0, quadraticApproximation.coneConstraint);
    ASSERT_LT(std::abs(y_eps - y_quadmodel), tol);
  }
}

TEST(TestFrictionConeConstraint, negativeDefinite_body_frame) {
  friction_cone::Config config;

  const scalar_t tol = 1e-12;
  const int N = 10000;

  for (int i = 0; i < N; ++i) {
    // Random forces, orientation, and terrain direction
    const vector3_t forcesInBodyFrame = 100.0 * vector3_t::Random();
    const vector3_t eulerXYZ = vector3_t::Random();
    const matrix3_t t_R_w = rotationMatrixOriginToBase<scalar_t>(vector3_t::Random());

    // Compute additional inputs consistently
    const matrix3_t w_R_b = rotationMatrixBaseToOrigin(eulerXYZ);
    const vector3_t forcesInTerrainFrame = t_R_w * w_R_b * forcesInBodyFrame;

    auto quadraticApproximation = frictionConeDerivatives(config, forcesInTerrainFrame, t_R_w, w_R_b, eulerXYZ, forcesInBodyFrame);

    matrix_t hessian(6, 6);
    hessian << quadraticApproximation.d2Cone_deuler2, quadraticApproximation.d2Cone_dudeuler.transpose(),
        quadraticApproximation.d2Cone_dudeuler, quadraticApproximation.d2Cone_du2;
    ASSERT_LT(ocs2::LinearAlgebra::symmetricEigenvalues(hessian).maxCoeff(), tol);
  }
}