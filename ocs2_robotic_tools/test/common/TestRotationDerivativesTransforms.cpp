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

#include <ocs2_core/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using Quaternion_t = Eigen::Quaternion<scalar_t>;
using AngleAxis_t = Eigen::AngleAxis<scalar_t>;

TEST(RotationDerivativesTransforms, global) {
  const vector3_t eulerAngles(0.1, 0.2, 0.3);
  const vector3_t eulerAngleDerivatives(0.4, 0.5, 0.6);

  // Check mapping vs. direct
  const matrix3_t mapping = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  const vector3_t angularVelocity = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives(eulerAngles, eulerAngleDerivatives);
  ASSERT_TRUE(angularVelocity.isApprox(mapping * eulerAngleDerivatives));

  // Check mapping back
  const vector3_t eulerAngleDerivativesRecovered = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity(eulerAngles, angularVelocity);
  ASSERT_TRUE(eulerAngleDerivativesRecovered.isApprox(eulerAngleDerivatives));
}

TEST(RotationDerivativesTransforms, local) {
  const vector3_t eulerAngles(0.1, 0.2, 0.3);
  const vector3_t eulerAngleDerivatives(0.4, 0.5, 0.6);

  // Check mapping vs. direct
  const matrix3_t mapping = getMappingFromEulerAnglesZyxDerivativeToLocalAngularVelocity(eulerAngles);
  const vector3_t angularVelocity = getLocalAngularVelocityFromEulerAnglesZyxDerivatives(eulerAngles, eulerAngleDerivatives);
  ASSERT_TRUE(angularVelocity.isApprox(mapping * eulerAngleDerivatives));

  // Check mapping back
  const vector3_t eulerAngleDerivativesRecovered = getEulerAnglesZyxDerivativesFromLocalAngularVelocity(eulerAngles, angularVelocity);
  ASSERT_TRUE(eulerAngleDerivativesRecovered.isApprox(eulerAngleDerivatives));
}

TEST(RotationDerivativesTransforms, localVsGlobal) {
  const vector3_t eulerAngles(0.1, 0.2, 0.3);
  const vector3_t eulerAngleDerivatives(0.4, 0.5, 0.6);
  const matrix3_t R = getRotationMatrixFromZyxEulerAngles(eulerAngles);

  const vector3_t localAngularVelocity = getLocalAngularVelocityFromEulerAnglesZyxDerivatives(eulerAngles, eulerAngleDerivatives);
  const vector3_t globalAngularVelocity = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives(eulerAngles, eulerAngleDerivatives);

  ASSERT_TRUE(globalAngularVelocity.isApprox(R * localAngularVelocity));
}