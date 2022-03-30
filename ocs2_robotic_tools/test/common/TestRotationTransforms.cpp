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
    ASSERT_NEAR((quaternionDistance(q, qRef) - adInterface.getFunctionValue(qDyn, qRefDyn)).norm(), 0, 1e-5);

    matrix_t jacManual = quaternionDistanceJacobian(q, qRef);
    matrix_t jacAuto = adInterface.getJacobian(qDyn, qRefDyn);
    ASSERT_NEAR((jacManual - jacAuto).sum(), 0, 1e-5);
  }

  ASSERT_TRUE(true);
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
    ASSERT_NEAR(quaternionDistance(q, resultQuaternion).norm(), 0, 1e-5)
        << "q = " << q.coeffs().transpose() << ", resultQuaternion = " << resultQuaternion.coeffs().transpose();
  }
}

