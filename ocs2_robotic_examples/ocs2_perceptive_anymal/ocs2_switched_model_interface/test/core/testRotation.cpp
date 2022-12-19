//
// Created by rgrandia on 14.07.20.
//

#include <gtest/gtest.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include "ocs2_switched_model_interface/core/Rotations.h"

using namespace switched_model;

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrixBaseToOrigin_check(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  // inputs are the intrinsic rotation angles in RADIANTS
  SCALAR_T sinAlpha = sin(eulerAnglesXYZ(0));
  SCALAR_T cosAlpha = cos(eulerAnglesXYZ(0));
  SCALAR_T sinBeta = sin(eulerAnglesXYZ(1));
  SCALAR_T cosBeta = cos(eulerAnglesXYZ(1));
  SCALAR_T sinGamma = sin(eulerAnglesXYZ(2));
  SCALAR_T cosGamma = cos(eulerAnglesXYZ(2));

  Eigen::Matrix<SCALAR_T, 3, 3> Rx, Ry, Rz;
  Rx << SCALAR_T(1), SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), cosAlpha, -sinAlpha, SCALAR_T(0), sinAlpha, cosAlpha;
  Ry << cosBeta, SCALAR_T(0), sinBeta, SCALAR_T(0), SCALAR_T(1), SCALAR_T(0), -sinBeta, SCALAR_T(0), cosBeta;
  Rz << cosGamma, -sinGamma, SCALAR_T(0), sinGamma, cosGamma, SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), SCALAR_T(1);

  return Rx * Ry * Rz;
}

TEST(testRotations, rotationMatrix) {
  for (int i = 0; i < 1000; i++) {
    vector3_t eulerAngles = vector3_t::Random();

    const auto o_R_b_check = rotationMatrixBaseToOrigin_check(eulerAngles);
    const auto o_R_b = rotationMatrixBaseToOrigin(eulerAngles);

    ASSERT_NEAR((o_R_b_check - o_R_b).norm(), 0, 1e-12);
  }
}

TEST(testRotations, rotateVector) {
  for (int i = 0; i < 1000; i++) {
    const vector3_t eulerAngles = vector3_t::Random();
    const vector3_t vector = vector3_t::Random();

    const vector3_t o_b_check = rotationMatrixBaseToOrigin_check(eulerAngles) * vector;
    const vector3_t o_b_vector = rotateVectorBaseToOrigin(vector, eulerAngles);
    ASSERT_NEAR((o_b_check - o_b_vector).norm(), 0, 1e-12);

    const vector3_t b_o_check = rotationMatrixBaseToOrigin_check(eulerAngles).transpose() * vector;
    const vector3_t b_o_vector = rotateVectorOriginToBase(vector, eulerAngles);
    ASSERT_NEAR((b_o_check - b_o_vector).norm(), 0, 1e-12);
  }
}

TEST(testRotations, rotationDerivative) {
  using ad_vector_t = ocs2::CppAdInterface::ad_vector_t;
  using ad_scalar_t = ocs2::CppAdInterface::ad_scalar_t;

  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    vector3_ad_t eulerAngles = x.head<3>();
    vector3_ad_t v_base = p.head<3>();
    y = rotateVectorBaseToOrigin(v_base, eulerAngles);
  };

  ocs2::CppAdInterface adInterface(adFunction, 3, 3, "rotationDerivativeTest");
  adInterface.createModels();

  for (int i = 0; i < 1000; i++) {
    vector3_t eulerAngles = vector3_t::Random();
    vector3_t v_base = vector3_t::Random();

    matrix3_t jacManual = rotationBaseToOriginJacobian(eulerAngles, v_base);
    Eigen::MatrixXd jacAuto = adInterface.getJacobian(eulerAngles, v_base);
    ASSERT_NEAR((jacManual - jacAuto).array().abs().sum(), 0, 1e-12);
  }

  ASSERT_TRUE(true);
}

TEST(testRotations, eulerRates) {
  for (int i = 0; i < 1000; i++) {
    const vector3_t eulerAngles = vector3_t::Random();
    const vector3_t angularVelocityInBase = vector3_t::Random();

    const vector3_t eulerRate_check = angularVelocitiesToEulerAngleDerivativesMatrix(eulerAngles) * angularVelocityInBase;
    const vector3_t eulerRate = angularVelocitiesToEulerAngleDerivatives(angularVelocityInBase, eulerAngles);
    ASSERT_NEAR((eulerRate_check - eulerRate).norm(), 0, 1e-12);
  }
}
