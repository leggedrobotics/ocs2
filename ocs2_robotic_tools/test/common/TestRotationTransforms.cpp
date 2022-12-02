/******************************************************************************
Copyright (c) 2020, Johannes Pankert. All rights reserved.

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

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <Eigen/Dense>

using namespace ocs2;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector3_ad_t = Eigen::Matrix<ad_scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using Quaternion_t = Eigen::Quaternion<scalar_t>;
using Quaternion_ad_t = Eigen::Quaternion<ad_scalar_t>;
using AngleAxis_t = Eigen::AngleAxis<scalar_t>;

TEST(RotationTransforms, quaternionDifferenceJacobian) {
  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Quaternion_ad_t q(x.head<4>());
    Quaternion_ad_t qRef(p.head<4>());
    y.resize(3);
    y = quaternionDistance(q, qRef);
  };

  CppAdInterface adInterface(adFunction, 4, 4, "quaternion_error");
  adInterface.createModels();

  for (int i = 0; i < 1000; i++) {
    Quaternion_t q = Quaternion_t::UnitRandom();
    Quaternion_t qRef = Quaternion_t::UnitRandom();

    vector_t qDyn = q.coeffs();
    vector_t qRefDyn = qRef.coeffs();
    ASSERT_TRUE(quaternionDistance(q, qRef).isApprox(adInterface.getFunctionValue(qDyn, qRefDyn), 1e-5));

    matrix_t jacManual = quaternionDistanceJacobian(q, qRef);
    matrix_t jacAuto = adInterface.getJacobian(qDyn, qRefDyn);
    ASSERT_TRUE(jacManual.isApprox(jacAuto, 1e-5));
  }
}

// Asserts that the quaterion performs the same rotation as the rotation matrix
void testQuaternionAgainstRotationMatrix(const Quaternion_t& quat, const matrix3_t& rotMatrix) {
  ASSERT_TRUE(quat.toRotationMatrix().isApprox(rotMatrix, 1e-5)) << "rotMatrix = \n"
                                                                 << rotMatrix << "\n quat = " << quat.coeffs().transpose();
}

// Attempts to test all the branches in the cpp ad function
TEST(RotationTransforms, matrixToQuaternionCppAd) {
  auto adFunction = [](const ad_vector_t& x, ad_vector_t& y) {
    Eigen::Map<const ad_matrix_t> rotMatrix(x.data(), 3, 3);
    y.resize(4);
    y = matrixToQuaternion(rotMatrix).coeffs();
  };

  CppAdInterface adInterface(adFunction, 9, "matrix_to_quaternion");
  adInterface.createModels();

  auto testRotationMatrix = [&](const matrix3_t& rotMatrix) -> Quaternion_t {
    Eigen::Map<const vector_t> serializedRotMatrix(rotMatrix.data(), 9, 1);
    const vector_t result = adInterface.getFunctionValue(serializedRotMatrix);
    const Quaternion_t resultQuaternion(result[3], result[0], result[1], result[2]);
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);
    return resultQuaternion;
  };

  // No rotation
  testRotationMatrix(matrix3_t::Identity());

  // 90 deg rotations
  testRotationMatrix(AngleAxis_t(0.5 * M_PI, vector3_t::UnitX()).toRotationMatrix());
  testRotationMatrix(AngleAxis_t(0.5 * M_PI, vector3_t::UnitY()).toRotationMatrix());
  testRotationMatrix(AngleAxis_t(0.5 * M_PI, vector3_t::UnitZ()).toRotationMatrix());

  // 180 deg rotations
  testRotationMatrix(AngleAxis_t(M_PI, vector3_t::UnitX()).toRotationMatrix());
  testRotationMatrix(AngleAxis_t(M_PI, vector3_t::UnitY()).toRotationMatrix());
  testRotationMatrix(AngleAxis_t(M_PI, vector3_t::UnitZ()).toRotationMatrix());

  // Random
  for (int i = 0; i < 1000; i++) {
    const Quaternion_t q = Quaternion_t::UnitRandom();
    auto resultQuaternion = testRotationMatrix(q.toRotationMatrix());

    // Assert that our original generating quaternion matches the resulting one
    ASSERT_TRUE(quaternionDistance(q, resultQuaternion).isZero(1e-5))
        << "q = " << q.coeffs().transpose() << ", resultQuaternion = " << resultQuaternion.coeffs().transpose();
  }
}

TEST(RotationTransforms, rotationMatrixToRotationVector) {
  auto runTest = [](const AngleAxis_t& angleAxis) {
    auto computedAngleAxis = rotationMatrixToRotationVector<scalar_t>(angleAxis.toRotationMatrix());
    const scalar_t vectol = 1e-6;
    return (computedAngleAxis - angleAxis.angle() * angleAxis.axis()).norm() < vectol;
  };
  auto message = [](const AngleAxis_t& angleAxis) {
    auto computedAngleAxis = rotationMatrixToRotationVector<scalar_t>(angleAxis.toRotationMatrix());
    std::stringstream ss;
    ss << "Failed test for angle: " << angleAxis.angle() << ", axle: " << angleAxis.axis().transpose()
       << "\n computed: " << computedAngleAxis.transpose() << ", should be " << (angleAxis.angle() * angleAxis.axis()).transpose();
    return ss.str();
  };

  // Test edge cases
  const scalar_t smallAngle = 1e-12;
  const scalar_t smallAngle2 = 1e-6;
  const scalar_array_t edgeCases = {0.0, smallAngle, smallAngle2, M_PI_2, M_PI - smallAngle2, M_PI - smallAngle, M_PI};
  const std::vector<vector3_t> testDirections = {vector3_t::UnitX(), vector3_t::UnitY(), vector3_t::UnitZ(),
                                                 vector3_t::Random().normalized()};

  for (auto angle : edgeCases) {
    for (const auto& axis : testDirections) {
      EXPECT_TRUE(runTest({angle, axis})) << message({angle, axis});
      EXPECT_TRUE(runTest({angle, -axis})) << message({angle, -axis});
    }
  }

  // Test range of angles
  const scalar_t dAngle = 1e-4;
  for (scalar_t angle = 0.0; angle < M_PI; angle += dAngle) {
    for (const auto& axis : testDirections) {
      EXPECT_TRUE(runTest({angle, axis})) << message({angle, axis});
      EXPECT_TRUE(runTest({angle, -axis})) << message({angle, -axis});
    }
  }
}

TEST(RotationTransforms, rotationErrorSign) {
  // This just tests the sign correctness of the error. Edge cases are tested in rotationMatrixToRotationVector

  // Pick random rotations
  Quaternion_t lhsToWorld = Quaternion_t(0.1, 0.2, 0.3, 0.4).normalized();
  Quaternion_t rhsToWorld = Quaternion_t(0.5, 0.6, 0.7, 0.8).normalized();

  // Compute rotation error in world
  const auto errorInWorld = rotationErrorInWorld(lhsToWorld.toRotationMatrix(), rhsToWorld.toRotationMatrix());
  const AngleAxis_t errorAngleAxis(errorInWorld.norm(), errorInWorld.normalized());

  // Test for correct composition of the error
  // Error = lhs [-] rhs => lhs = error [+] rhs
  // Comparing as rotation removes ambiguity between the two quaternions that represent the same rotation.
  ASSERT_TRUE(lhsToWorld.toRotationMatrix().isApprox((errorAngleAxis * rhsToWorld).toRotationMatrix()));
}

TEST(RotationTransforms, rotationErrorGlobalvsLocal) {
  // This just tests the frame conventions of the error. Edge cases are tested in rotationMatrixToRotationVector

  // Pick random rotations
  Quaternion_t lhsToWorld = Quaternion_t(0.1, 0.2, 0.3, 0.4).normalized();
  Quaternion_t rhsToWorld = Quaternion_t(0.5, 0.6, 0.7, 0.8).normalized();

  // Compute independently in world and local frame
  const auto errorInWorld = rotationErrorInWorld(lhsToWorld.toRotationMatrix(), rhsToWorld.toRotationMatrix());
  const auto errorInLocal = rotationErrorInLocal(lhsToWorld.toRotationMatrix(), rhsToWorld.toRotationMatrix());

  // Test for equality in world frame. Both lhs and rhs frames are valid for the local representation.
  ASSERT_TRUE(errorInWorld.isApprox(lhsToWorld * errorInLocal));
  ASSERT_TRUE(errorInWorld.isApprox(rhsToWorld * errorInLocal));
}

TEST(RotationTransforms, rotationErrorGradientDescent) {
  // Tests that the box minus rotation error has suitable gradients
  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    vector3_ad_t eulerAngles = x.head<3>();
    vector3_ad_t eulerAnglesReference = p.head<3>();
    y = rotationErrorInWorld(getRotationMatrixFromZyxEulerAngles(eulerAngles), getRotationMatrixFromZyxEulerAngles(eulerAnglesReference));
  };

  CppAdInterface adInterface(adFunction, 3, 3, "rotationErrorJacobianTest");
  adInterface.createModels();

  // Check Jacobian at origin. The Jacobian is not identity because the ZYX euler angles are stored in z - y - x order...
  const matrix3_t originJacobian = (matrix3_t() << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0).finished();

  matrix3_t jacOrigin = adInterface.getJacobian(vector3_t::Zero(), vector3_t::Zero());
  ASSERT_TRUE(jacOrigin.isApprox(originJacobian));

  // Check Jacobian at full rotation
  matrix3_t jac2Pi = adInterface.getJacobian(vector3_t::Zero(), vector3_t{0.0, 0.0, 2.0 * M_PI});
  ASSERT_TRUE(jac2Pi.isApprox(originJacobian));

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
  ASSERT_TRUE(gradientDescent(eulerAngles, eulerAnglesReference).isApprox(eulerAnglesReference, 1e-6));

  // Able to do gradient descent between random angles
  eulerAngles = vector3_t::Random();
  eulerAnglesReference = vector3_t::Random();
  ASSERT_TRUE(gradientDescent(eulerAngles, eulerAnglesReference).isApprox(eulerAnglesReference, 1e-6));
}

TEST(RotationTransforms, rotationErrorSquaredGradient) {
  // Tests that the box minus rotation error has suitable, finite gradients when tacking the error squared
  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    ad_matrix_t R(3, 3);
    R << x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8];
    ad_vector_t f = rotationMatrixToRotationVector<ad_scalar_t>(R);
    y = ad_vector_t(1);
    y << f.squaredNorm();
  };

  CppAdInterface adInterface(adFunction, 9, 0, "rotationErrorSquaredTest");
  adInterface.createModels();

  vector_t in(9);
  in << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  const auto Jidentity = adInterface.getJacobian(in);
  ASSERT_TRUE(Jidentity.allFinite());

  in << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  const auto J180 = adInterface.getJacobian(in);
  ASSERT_TRUE(J180.allFinite());
}

TEST(AngleModulo, testSetOfAngles) {
  // test if two angles have a multiple of 2*pi between them
  auto anglesAreEqual = [](scalar_t x, scalar_t y, scalar_t tol) {
    const scalar_t diff = std::abs(x - y);
    const scalar_t diffLoops = diff / (2.0 * M_PI);
    return std::abs(diffLoops - std::round(diffLoops)) < (tol / (2.0 * M_PI));
  };

  std::vector<scalar_t> testRange = {-123.0, -17.0 / 3.0, -3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 17.0 / 3.0, 123.0};
  std::for_each(testRange.begin(), testRange.end(), [](scalar_t& v) { v *= M_PI; });

  const scalar_t tol = 1e-12;

  for (const scalar_t x : testRange) {
    for (const scalar_t ref : testRange) {
      const auto y = moduloAngleWithReference(x, ref);

      // Check that y is within bound around reference
      ASSERT_LE(y, ref + M_PI);
      ASSERT_GE(y, ref - M_PI);

      // Check that the angle represented by y is still equal to x
      ASSERT_TRUE(anglesAreEqual(x, y, tol));
    }
  }
}
