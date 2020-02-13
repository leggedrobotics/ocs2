
#include <iostream>
#include <gtest/gtest.h>

#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

using namespace ocs2;

TEST(testModelDataBase, testModelDataLinearInterpolation) {
  // create data
  const size_t N = 10;
  std::vector<double> timeArray(N);
  std::vector<ModelDataBase, Eigen::aligned_allocator<ModelDataBase>> modelDataBaseArray(N);

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
