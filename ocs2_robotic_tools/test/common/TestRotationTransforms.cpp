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

TEST(RotationTransforms, quaternionDifferenceJacobian) {
  using ad_vector_t = ocs2::ad_vector_t;
  using ad_scalar_t = ocs2::ad_scalar_t;

  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Quaternion<ad_scalar_t> q(x.head<4>());
    Eigen::Quaternion<ad_scalar_t> qRef(p.head<4>());
    y.resize(3);
    y = ocs2::quaternionDistance(q, qRef);
  };

  ocs2::CppAdInterface adInterface(adFunction, 4, 4, "quaternion_error");
  adInterface.createModels();

  for (int i = 0; i < 1000; i++) {
    Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
    Eigen::Quaterniond qRef = Eigen::Quaterniond::UnitRandom();

    Eigen::VectorXd qDyn = q.coeffs();
    Eigen::VectorXd qRefDyn = qRef.coeffs();
    ASSERT_NEAR((ocs2::quaternionDistance(q, qRef) - adInterface.getFunctionValue(qDyn, qRefDyn)).norm(), 0, 1e-5);

    Eigen::MatrixXd jacManual = ocs2::quaternionDistanceJacobian(q, qRef);
    Eigen::MatrixXd jacAuto = adInterface.getJacobian(qDyn, qRefDyn);
    ASSERT_NEAR((jacManual - jacAuto).sum(), 0, 1e-5);
  }

  ASSERT_TRUE(true);
}

// Asserts that the quaterion performs the same rotation as the rotation matrix
void testQuaternionAgainstRotationMatrix(const Eigen::Quaterniond& quat, const Eigen::Matrix3d& rotMatrix){
  for (int i = 0; i < 10; i++){
    const Eigen::Vector3d preRotatePoint = Eigen::Vector3d::Random();
    const Eigen::Vector3d postRotatePointMatrix = rotMatrix*preRotatePoint;
    const Eigen::Vector3d postRotatePointQuat = quat*preRotatePoint;
    ASSERT_NEAR((postRotatePointMatrix - postRotatePointQuat).norm(), 0, 1e-5)<<"rotMatrix = \n"<<rotMatrix<<"\n quat = "<<quat.coeffs().transpose();
  }
}

//Attempts to test all the branches in the cpp ad function
TEST(RotationTransforms, matrixToQuaternionCppAdSelected) {
  using ad_matrix_t = ocs2::ad_matrix_t;
  using ad_vector_t = ocs2::ad_vector_t;
  using ad_scalar_t = ocs2::ad_scalar_t;

  auto adFunction = [](const ad_vector_t& x,  ad_vector_t& y) {
    ad_matrix_t rotMatrix = ad_matrix_t::Zero(3,3);
    rotMatrix << x;
    y.resize(4);
    y = ocs2::matrixToQuaternion(rotMatrix).coeffs();
  };

  ocs2::CppAdInterface adInterface(adFunction, 9, "matrix_to_quaternion");
  adInterface.createModels();

  {
    Eigen::Matrix3d rotMatrix;
    // clang-format off
    rotMatrix << 1, 0, 0,
                 0, 1, 0,
                 0, 0, 1;
    // clang format on
    Eigen::VectorXd serializedRotMatrix(9);
    serializedRotMatrix << rotMatrix;
    const Eigen::VectorXd result = adInterface.getFunctionValue(serializedRotMatrix);
    const Eigen::Quaterniond resultQuaternion(result[3],result[0],result[1],result[2]);
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);
  }

  {
    Eigen::Matrix3d rotMatrix;
    // clang-format off
    rotMatrix << 0, 0, 1,
                 0, 1, 0,
                -1, 0, 0;
    // clang format on
    Eigen::VectorXd serializedRotMatrix(9);
    serializedRotMatrix << rotMatrix;
    const Eigen::VectorXd result = adInterface.getFunctionValue(serializedRotMatrix);
    const Eigen::Quaterniond resultQuaternion(result[3],result[0],result[1],result[2]);
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);
  }

  {
    Eigen::Matrix3d rotMatrix;
    // clang-format off
    rotMatrix << 0, 0, 1,
                 0, 1, 0,
                -1, 0, 0;
    // clang format on
    Eigen::VectorXd serializedRotMatrix(9);
    serializedRotMatrix << rotMatrix;
    const Eigen::VectorXd result = adInterface.getFunctionValue(serializedRotMatrix);
    const Eigen::Quaterniond resultQuaternion(result[3],result[0],result[1],result[2]);
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);
  }

  {
    Eigen::Matrix3d rotMatrix;
    // clang-format off
    rotMatrix << 1, 0, 0,
                 0, 0, 1,
                 0,-1, 0;
    // clang format on
    Eigen::VectorXd serializedRotMatrix(9);
    serializedRotMatrix << rotMatrix;
    const Eigen::VectorXd result = adInterface.getFunctionValue(serializedRotMatrix);
    const Eigen::Quaterniond resultQuaternion(result[3],result[0],result[1],result[2]);
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);
  }

  {
    Eigen::Matrix3d rotMatrix;
    // clang-format off
    rotMatrix << 0, 1, 0,
                -1, 0, 0,
                 0, 0, 1;
    // clang format on
    Eigen::VectorXd serializedRotMatrix(9);
    serializedRotMatrix << rotMatrix;
    const Eigen::VectorXd result = adInterface.getFunctionValue(serializedRotMatrix);
    const Eigen::Quaterniond resultQuaternion(result[3],result[0],result[1],result[2]);
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);
  }

  {
    Eigen::Matrix3d rotMatrix;
    // clang-format off
    rotMatrix << 0,-1, 0,
                -1, 0, 0,
                 0, 0,-1;
    // clang format on
    Eigen::VectorXd serializedRotMatrix(9);
    serializedRotMatrix << rotMatrix;
    const Eigen::VectorXd result = adInterface.getFunctionValue(serializedRotMatrix);
    const Eigen::Quaterniond resultQuaternion(result[3],result[0],result[1],result[2]);
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);
  }
}

TEST(RotationTransforms, matrixToQuaternionCppAdRandom) {
  using ad_matrix_t = ocs2::ad_matrix_t;
  using ad_vector_t = ocs2::ad_vector_t;
  using ad_scalar_t = ocs2::ad_scalar_t;

  auto adFunction = [](const ad_vector_t& x,  ad_vector_t& y) {
    ad_matrix_t rotMatrix = ad_matrix_t::Zero(3,3);
    rotMatrix << x;
    y.resize(4);
    y = ocs2::matrixToQuaternion(rotMatrix).coeffs();
  };

  ocs2::CppAdInterface adInterface(adFunction, 9, "matrix_to_quaternion");
  adInterface.createModels();

  for (int i = 0; i < 1000; i++) {
    const Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();

    const auto rotMatrix = q.toRotationMatrix();
    Eigen::VectorXd serializedRotMatrix(9);
    serializedRotMatrix << rotMatrix;

    const Eigen::VectorXd result = adInterface.getFunctionValue(serializedRotMatrix);
    const Eigen::Quaterniond resultQuaternion(result[3],result[0],result[1],result[2]);

    // Assert that our original generating quaternion matches the resulting one
    ASSERT_NEAR(ocs2::quaternionDistance(q, resultQuaternion).norm(), 0, 1e-5)<<"q = "<<q.coeffs().transpose()<<", resultQuaternion = "<<resultQuaternion.coeffs().transpose();

    // Assert that our resulting quaternion performs the same rotation as the rotation matrix
    testQuaternionAgainstRotationMatrix(resultQuaternion, rotMatrix);

  }
}
