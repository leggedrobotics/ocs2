/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

TEST(testModelDataBase, testModelDataLinearInterpolation) {
  // create data
  const size_t N = 10;
  std::vector<double> timeArray(N);
  ModelDataBase::array_t modelDataBaseArray(N);

  for (size_t i = 0; i < N; i++) {
	double t = 2.0 * i;
	timeArray[i] = t;
	modelDataBaseArray[i].time_ = t;
	modelDataBaseArray[i].dynamics_ = Eigen::Vector3d::Ones() * t;
	modelDataBaseArray[i].dynamicsStateDerivative_ = Eigen::Matrix3d::Ones() * t;
  }

  double time = 5.0;
  // get (index, alpha) pair
  const auto indexAlpha = ModelData::timeSegment(time, &timeArray);
  // scalar
  double enquiryScalar;
  ModelData::interpolate(indexAlpha, enquiryScalar, &modelDataBaseArray, ModelData::time);
  // dynamic vector
  Eigen::VectorXd enquiryVector;
  ModelData::interpolate(indexAlpha, enquiryVector, &modelDataBaseArray, ModelData::dynamics);
  // dynamic matrix
  Eigen::MatrixXd enquiryMatrix;
  ModelData::interpolate(indexAlpha, enquiryMatrix, &modelDataBaseArray, ModelData::dynamicsStateDerivative);

  ASSERT_TRUE(enquiryScalar == time);
  ASSERT_TRUE(enquiryVector.isApprox(Eigen::Vector3d::Ones()*time));
  ASSERT_TRUE(enquiryMatrix.isApprox(Eigen::Matrix3d::Ones()*time));
}

TEST(testModelDataBase, testMovableCopyable) {
  ASSERT_TRUE(std::is_copy_constructible<ModelDataBase>::value);
  ASSERT_TRUE(std::is_move_constructible<ModelDataBase>::value);
  ASSERT_TRUE(std::is_nothrow_move_constructible<ModelDataBase>::value);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
