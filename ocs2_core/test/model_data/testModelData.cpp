/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <iostream>
#include <type_traits>

#include <gtest/gtest.h>

#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

using namespace ocs2;

TEST(testModelData, testModelDataLinearInterpolation) {
  // create data
  const size_t N = 10;
  std::vector<double> timeArray(N);
  std::vector<ModelData> modelDataBaseArray(N);

  for (size_t i = 0; i < N; i++) {
    double t = 2.0 * i;
    timeArray[i] = t;
    modelDataBaseArray[i].time = t;
    modelDataBaseArray[i].dynamics.f = Eigen::Vector3d::Ones() * t;
    modelDataBaseArray[i].dynamics.dfdx = Eigen::Matrix3d::Ones() * t;
  }

  double time = 5.0;
  // get (index, alpha) pair
  const auto indexAlpha = LinearInterpolation::timeSegment(time, timeArray);

  const scalar_t enquiryScalar = LinearInterpolation::interpolate(indexAlpha, modelDataBaseArray, model_data::time);
  const vector_t enquiryVector = LinearInterpolation::interpolate(indexAlpha, modelDataBaseArray, model_data::dynamics_f);
  const matrix_t enquiryMatrix = LinearInterpolation::interpolate(indexAlpha, modelDataBaseArray, model_data::dynamics_dfdx);

  EXPECT_TRUE(enquiryScalar == time);
  EXPECT_TRUE(enquiryVector.isApprox(Eigen::Vector3d::Ones() * time));
  EXPECT_TRUE(enquiryMatrix.isApprox(Eigen::Matrix3d::Ones() * time));
}

TEST(testModelData, testMovableCopyable) {
  ASSERT_TRUE(std::is_copy_constructible<ModelData>::value);
  ASSERT_TRUE(std::is_move_constructible<ModelData>::value);
  ASSERT_TRUE(std::is_nothrow_move_constructible<ModelData>::value);
}
