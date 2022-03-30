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

//////  Rotation Error Tests
TEST(testRotations, rotationError) {
  // This just tests the sign correctness of the error. Edge cases are tested in rotationMatrixToAngleAxis

  // Pick random rotations
  Eigen::Quaternion<scalar_t> lhsToWorld = Eigen::Quaternion<scalar_t>(0.1, 0.2, 0.3, 0.4).normalized();
  Eigen::Quaternion<scalar_t> rhsToWorld = Eigen::Quaternion<scalar_t>(0.5, 0.6, 0.7, 0.8).normalized();

  // Compute rotation error in world
  const auto errorInWorld = rotationErrorInWorld(lhsToWorld.toRotationMatrix(), rhsToWorld.toRotationMatrix());
  const Eigen::AngleAxis<scalar_t> errorAngleAxis(errorInWorld.norm(), errorInWorld.normalized());

  // Test for correct composition of the error
  // Error = lhs [-] rhs => lhs = error [+] rhs
  // Comparing as rotation removes ambiguity between the two quaternions that represent the same rotation.
  ASSERT_TRUE(lhsToWorld.toRotationMatrix().isApprox((errorAngleAxis * rhsToWorld).toRotationMatrix()));
}

TEST(testRotations, rotationErrorGlobalvsLocal) {
  // This just tests the frame conventions of the error. Edge cases are tested in rotationMatrixToAngleAxis

  // Pick random rotations
  Eigen::Quaternion<scalar_t> lhsToWorld = Eigen::Quaternion<scalar_t>(0.1, 0.2, 0.3, 0.4).normalized();
  Eigen::Quaternion<scalar_t> rhsToWorld = Eigen::Quaternion<scalar_t>(0.5, 0.6, 0.7, 0.8).normalized();

  // Compute independently in world and local frame
  const auto errorInWorld = rotationErrorInWorld(lhsToWorld.toRotationMatrix(), rhsToWorld.toRotationMatrix());
  const auto errorInLocal = rotationErrorInLocal(lhsToWorld.toRotationMatrix(), rhsToWorld.toRotationMatrix());

  // Test for equality in world frame. Both lhs and rhs frames are valid for the local representation.
  ASSERT_TRUE(errorInWorld.isApprox(lhsToWorld * errorInLocal));
  ASSERT_TRUE(errorInWorld.isApprox(rhsToWorld * errorInLocal));
}

TEST(testRotations, rotationMatrixToAngleAxis) {
  auto runTest = [](const Eigen::AngleAxis<scalar_t>& angleAxis) {
    auto computedAngleAxis = rotationMatrixToAngleAxis<scalar_t>(angleAxis.toRotationMatrix());
    const scalar_t vectol = 1e-6;
    return (computedAngleAxis - angleAxis.angle() * angleAxis.axis()).norm() < vectol;
  };
  auto message = [](const Eigen::AngleAxis<scalar_t>& angleAxis) {
    auto computedAngleAxis = rotationMatrixToAngleAxis<scalar_t>(angleAxis.toRotationMatrix());
    std::stringstream ss;
    ss << "Failed test for angle: " << angleAxis.angle() << ", axle: " << angleAxis.axis().transpose()
       << "\n computed: " << computedAngleAxis.transpose() << ", should be " << (angleAxis.angle() * angleAxis.axis()).transpose();
    return ss.str();
  };

  // Test edge cases
  const scalar_t smallAngle = 1e-12;
  const scalar_t smallAngle2 = 1e-6;
  const ocs2::scalar_array_t edgeCases = {0.0, smallAngle, smallAngle2, M_PI_2, M_PI - smallAngle2, M_PI - smallAngle, M_PI};
  const vector3_t rotationUnitVector = vector3_t{1.0, -1.0, 3.0}.normalized();

  for (auto angle : edgeCases) {
    EXPECT_TRUE(runTest({angle, rotationUnitVector})) << message({angle, rotationUnitVector});
    EXPECT_TRUE(runTest({angle, -rotationUnitVector})) << message({angle, -rotationUnitVector});
  }

  // Test range of angles
  const scalar_t dAngle = 1e-4;
  for (scalar_t angle = 0.0; angle < M_PI; angle += dAngle) {
    const vector3_t axis = vector3_t::Random().normalized();  // random axis
    EXPECT_TRUE(runTest({angle, axis})) << message({angle, axis});
    EXPECT_TRUE(runTest({angle, -axis})) << message({angle, -axis});
  }
}

TEST(testRotations, rotationErrorGradientDescent) {
  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    vector3_ad_t eulerAngles = x.head<3>();
    vector3_ad_t eulerAnglesReference = p.head<3>();
    y = rotationErrorInWorldEulerXYZ(eulerAngles, eulerAnglesReference);
  };

  ocs2::CppAdInterface adInterface(adFunction, 3, 3, "rotationErrorJacobianTest");
  adInterface.createModels();

  // Check Jacobian at origin
  matrix3_t jacOrigin = adInterface.getJacobian(vector3_t::Zero(), vector3_t::Zero());
  ASSERT_LT((jacOrigin - matrix3_t::Identity()).norm(), 1e-6);

  // Check Jacobian at full rotation
  matrix3_t jac2Pi = adInterface.getJacobian(vector3_t::Zero(), vector3_t{0.0, 0.0, 2.0 * M_PI});
  ASSERT_LT((jac2Pi - matrix3_t::Identity()).norm(), 1e-6);

  auto gradientDescent = [&](const vector3_t& eulerAnglesStart, vector3_t eulerAnglesReference) {
    vector3_t eulerAngles = eulerAnglesStart;
    for (int i = 0; i < 1000; i++) {
      matrix3_t jac = adInterface.getJacobian(eulerAngles, eulerAnglesReference);
      vector3_t error = adInterface.getFunctionValue(eulerAngles, eulerAnglesReference);
      // Take step with fixed step size
      eulerAngles -= 0.1 * jac.fullPivHouseholderQr().solve(error);
    }
    return eulerAngles;
  };

  // Able to do gradient descent with exactly Pi offset
  vector3_t eulerAngles = vector3_t::Zero();
  vector3_t eulerAnglesReference = vector3_t{0.0, 0.0, M_PI};
  ASSERT_LT((gradientDescent(eulerAngles, eulerAnglesReference) - eulerAnglesReference).norm(), 1e-6);

  // Able to do gradient descent between random angles
  eulerAngles = vector3_t::Random();
  eulerAnglesReference = vector3_t::Random();
  ASSERT_LT((gradientDescent(eulerAngles, eulerAnglesReference) - eulerAnglesReference).norm(), 1e-6);
}
