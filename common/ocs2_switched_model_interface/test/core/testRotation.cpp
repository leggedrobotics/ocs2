//
// Created by rgrandia on 14.07.20.
//

#include <gtest/gtest.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include "ocs2_switched_model_interface/core/Rotations.h"

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
    switched_model::vector3_t eulerAngles = switched_model::vector3_t::Random();

    const auto o_R_b_check = rotationMatrixBaseToOrigin_check(eulerAngles);
    const auto o_R_b = switched_model::rotationMatrixBaseToOrigin(eulerAngles);

    ASSERT_NEAR((o_R_b_check - o_R_b).norm(), 0, 1e-12);
  }
}

TEST(testRotations, rotateVector) {
  for (int i = 0; i < 1000; i++) {
    const switched_model::vector3_t eulerAngles = switched_model::vector3_t::Random();
    const switched_model::vector3_t vector = switched_model::vector3_t::Random();

    const switched_model::vector3_t o_b_check = rotationMatrixBaseToOrigin_check(eulerAngles) * vector;
    const switched_model::vector3_t o_b_vector = switched_model::rotateVectorBaseToOrigin(vector, eulerAngles);
    ASSERT_NEAR((o_b_check - o_b_vector).norm(), 0, 1e-12);

    const switched_model::vector3_t b_o_check = rotationMatrixBaseToOrigin_check(eulerAngles).transpose() * vector;
    const switched_model::vector3_t b_o_vector = switched_model::rotateVectorOriginToBase(vector, eulerAngles);
    ASSERT_NEAR((b_o_check - b_o_vector).norm(), 0, 1e-12);
  }
}

TEST(testRotations, rotationDerivative) {
  using ad_vector_t = ocs2::CppAdInterface::ad_vector_t;
  using ad_scalar_t = ocs2::CppAdInterface::ad_scalar_t;

  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    switched_model::vector3_ad_t eulerAngles = x.head<3>();
    switched_model::vector3_ad_t v_base = p.head<3>();
    y = switched_model::rotateVectorBaseToOrigin(v_base, eulerAngles);
  };

  ocs2::CppAdInterface adInterface(adFunction, 3, 3, "rotationDerivativeTest");
  adInterface.createModels();

  for (int i = 0; i < 1000; i++) {
    switched_model::vector3_t eulerAngles = switched_model::vector3_t::Random();
    switched_model::vector3_t v_base = switched_model::vector3_t::Random();

    switched_model::matrix3_t jacManual = switched_model::rotationBaseToOriginJacobian(eulerAngles, v_base);
    Eigen::MatrixXd jacAuto = adInterface.getJacobian(eulerAngles, v_base);
    ASSERT_NEAR((jacManual - jacAuto).array().abs().sum(), 0, 1e-12);
  }

  ASSERT_TRUE(true);
}

TEST(testRotations, eulerRates) {
  for (int i = 0; i < 1000; i++) {
    const switched_model::vector3_t eulerAngles = switched_model::vector3_t::Random();
    const switched_model::vector3_t angularVelocityInBase = switched_model::vector3_t::Random();

    const switched_model::vector3_t eulerRate_check =
        switched_model::angularVelocitiesToEulerAngleDerivativesMatrix(eulerAngles) * angularVelocityInBase;
    const switched_model::vector3_t eulerRate =
        switched_model::angularVelocitiesToEulerAngleDerivatives(angularVelocityInBase, eulerAngles);
    ASSERT_NEAR((eulerRate_check - eulerRate).norm(), 0, 1e-12);
  }
}

TEST(testRotations, rotationMatrixToAngleAxis) {
  const ocs2::scalar_t vectol = 1e-6;

  auto runTest = [=](const Eigen::AngleAxis<ocs2::scalar_t>& angleAxis) {
    auto computedAngleAxis = switched_model::rotationMatrixToAngleAxis<ocs2::scalar_t>(angleAxis.toRotationMatrix());
    return (computedAngleAxis - angleAxis.angle() * angleAxis.axis()).norm() < vectol;
  };
  auto message = [=](const Eigen::AngleAxis<ocs2::scalar_t>& angleAxis) {
    auto computedAngleAxis = switched_model::rotationMatrixToAngleAxis<ocs2::scalar_t>(angleAxis.toRotationMatrix());
    std::stringstream ss;
    ss << "Failed test for angle: " << angleAxis.angle() << ", axle: " << angleAxis.axis().transpose()
       << "\n computed: " << computedAngleAxis.transpose() << ", should be " << (angleAxis.angle() * angleAxis.axis()).transpose();
    return ss.str();
  };

  // Test edge cases
  const ocs2::scalar_t smallAngle = 1e-12;
  const ocs2::scalar_t smallAngle2 = 1e-6;
  const ocs2::scalar_array_t edgeCases = {0.0, smallAngle, smallAngle2, M_PI_2, M_PI - smallAngle2, M_PI - smallAngle, M_PI};
  const switched_model::vector3_t rotationUnitVector = switched_model::vector3_t{1.0, -1.0, 3.0}.normalized();

  for (auto angle : edgeCases) {
    EXPECT_TRUE(runTest({angle, rotationUnitVector})) << message({angle, rotationUnitVector});
    EXPECT_TRUE(runTest({angle, -rotationUnitVector})) << message({angle, -rotationUnitVector});
  }

  // Test range of angles
  ocs2::scalar_t dAngle = 1e-4;
  for (ocs2::scalar_t angle = 0.0; angle < M_PI; angle += dAngle) {
    const switched_model::vector3_t axis = switched_model::vector3_t::Random().normalized();  // random axis
    EXPECT_TRUE(runTest({angle, axis})) << message({angle, axis});
    EXPECT_TRUE(runTest({angle, -axis})) << message({angle, -axis});
  }
}

TEST(testRotations, rotationErrorJacobian) {
  using ad_vector_t = ocs2::CppAdInterface::ad_vector_t;
  using ad_scalar_t = ocs2::CppAdInterface::ad_scalar_t;

  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    switched_model::vector3_ad_t eulerAngles = x.head<3>();
    switched_model::vector3_ad_t eulerAnglesReference = p.head<3>();
    y = switched_model::rotationError(switched_model::rotationMatrixOriginToBase(eulerAngles),
                                      switched_model::rotationMatrixOriginToBase(eulerAnglesReference));
  };

  ocs2::CppAdInterface adInterface(adFunction, 3, 3, "rotationErrorJacobianTest");
  adInterface.createModels();

  // Check Jacobian at origin
  switched_model::matrix3_t jacOrigin = adInterface.getJacobian(switched_model::vector3_t::Zero(), switched_model::vector3_t::Zero());
  ASSERT_LT((jacOrigin - switched_model::matrix3_t::Identity()).norm(), 1e-6);

  // Check Jacobian at full rotation
  switched_model::matrix3_t jac2Pi =
      adInterface.getJacobian(switched_model::vector3_t::Zero(), switched_model::vector3_t{0.0, 0.0, 2.0 * M_PI});
  ASSERT_LT((jac2Pi - switched_model::matrix3_t::Identity()).norm(), 1e-6);

  auto gradientDescent = [&](const switched_model::vector3_t& eulerAnglesStart, switched_model::vector3_t eulerAnglesReference) {
    switched_model::vector3_t eulerAngles = eulerAnglesStart;
    for (int i = 0; i < 1000; i++) {
      switched_model::matrix3_t jac = adInterface.getJacobian(eulerAngles, eulerAnglesReference);
      switched_model::vector3_t error = adInterface.getFunctionValue(eulerAngles, eulerAnglesReference);
      eulerAngles = eulerAngles - 0.1 * jac.fullPivHouseholderQr().solve(error);
    }
    return eulerAngles;
  };

  // Able to do gradient descent with exactly Pi offset
  switched_model::vector3_t eulerAngles = switched_model::vector3_t::Zero();
  switched_model::vector3_t eulerAnglesReference = switched_model::vector3_t{0.0, 0.0, M_PI};
  ASSERT_LT((gradientDescent(eulerAngles, eulerAnglesReference) - eulerAnglesReference).norm(), 1e-6);

  // Able to do gradient descent between random angles
  eulerAngles = switched_model::vector3_t::Random();
  eulerAnglesReference = switched_model::vector3_t::Random();
  ASSERT_LT((gradientDescent(eulerAngles, eulerAnglesReference) - eulerAnglesReference).norm(), 1e-6);
}