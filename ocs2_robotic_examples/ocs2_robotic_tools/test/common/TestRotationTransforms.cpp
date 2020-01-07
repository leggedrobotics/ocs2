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
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <Eigen/Dense>


TEST(RotationTransforms, quaternionDifferenceJacobian) {
  using ad_vector_t = ocs2::CppAdInterface<double>::ad_dynamic_vector_t;
  using ad_scalar_t = ocs2::CppAdInterface<double>::ad_scalar_t;

  auto adFunction = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Quaternion<ad_scalar_t> q(x.head<4>());
    Eigen::Quaternion<ad_scalar_t> qRef(p.head<4>());
    y.resize(3);
    y = ocs2::quaternionDistance(q, qRef);
  };

  ocs2::CppAdInterface<double> adInterface(adFunction, 3, 4, 4, "quaternion_error");
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}