

#include <gtest/gtest.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <Eigen/Dense>
#include <iostream>

TEST(testLinearInterpolation, testInterpolation) {
  using Data_T = Eigen::Matrix<double, 2,1>;

  // Create data
  std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 3.0, 4.0};
  std::vector<Data_T, Eigen::aligned_allocator<Data_T>> v;
  for (auto& t_k : t) {
    v.emplace_back(t_k * Data_T::Ones());
  }

  std::cout << "TEST DATA" << std::endl;
  for (int k=0; k<t.size(); k++){
    std::cout << "time: " << t[k] << " v: " << v[k].transpose() << std::endl;
  }

  // Test function
  auto test_interpolation = [&](double time, int index, double value){
    Data_T v_t;
    auto indexAlpha = ocs2::EigenLinearInterpolation<Data_T>::interpolate(time, v_t, &t, &v);
    auto foundIndex = indexAlpha.first;
    std::cout << "time: " << time << " index: " << foundIndex << " v: " << v_t.transpose() << std::endl;
    ASSERT_EQ(foundIndex, index);
    ASSERT_DOUBLE_EQ(v_t(0), value);
  };

  // Before start
  test_interpolation(-1.0, 0, 0.0);
  // At start
  test_interpolation(t[0], 0, t[0]);
  // First interval
  test_interpolation(0.5*t[0] + 0.5 *t[1], 0, 0.5*t[0] + 0.5 *t[1]);
  // Boundary to second interval
  test_interpolation(t[1], 0, t[1]);
  // Last interval
  test_interpolation( 0.5*t[4] + 0.5*t[5], 4,  0.5*t[4] + 0.5*t[5]);
  // At End
  test_interpolation( t[5], 4, t[5]);
  // Beyond end
  test_interpolation( t[5] + 1.0, 4, t[5]);
}

TEST(testLinearInterpolation, testSizeOneTime) {
  using Data_T = Eigen::Matrix<double, 2,1>;

  // Create data
  std::vector<double> t = {1.0};
  std::vector<Data_T, Eigen::aligned_allocator<Data_T>> v;
  for (auto& t_k : t) {
    v.emplace_back(t_k * Data_T::Ones());
  }

  std::cout << "TEST DATA" << std::endl;
  for (int k=0; k<t.size(); k++){
    std::cout << "time: " << t[k] << " v: " << v[k].transpose() << std::endl;
  }

  // Test function
  auto test_interpolation = [&](double time, int index, double value){
    Data_T v_t;
    auto indexAlpha = ocs2::EigenLinearInterpolation<Data_T>::interpolate(time, v_t, &t, &v);
    auto foundIndex = indexAlpha.first;
    std::cout << "time: " << time << " index: " << foundIndex << " v: " << v_t.transpose() << std::endl;
    ASSERT_EQ(foundIndex, index);
    ASSERT_DOUBLE_EQ(v_t(0), value);
  };

  // Before time
  test_interpolation(-1.0, 0, 1.0);
  // Beyond time
  test_interpolation( 2.0, 0, 1.0);
}

TEST(testLinearInterpolation, testDifferentEigenSizes) {
  using Data_T = Eigen::MatrixXd;
  std::vector<double> times = {0.0, 1.0};
  std::vector<Data_T, Eigen::aligned_allocator<Data_T>> data = {Data_T::Zero(2,3), Data_T::Ones(4,4)};

  Data_T result;
  ocs2::EigenLinearInterpolation<Data_T>::interpolate(0.4, result, &times, &data);
  bool test = result.isApprox(data[0]);
  ASSERT_TRUE(test);

  ocs2::EigenLinearInterpolation<Data_T>::interpolate(0.6, result, &times, &data);
  test = result.isApprox(data[1]);
  ASSERT_TRUE(test);

  ocs2::EigenLinearInterpolation<Data_T>::interpolate(-0.1, result, &times, &data);
  test = result.isApprox(data[0]);
  ASSERT_TRUE(test);

  ocs2::EigenLinearInterpolation<Data_T>::interpolate(1.1, result, &times, &data);
  test = result.isApprox(data[1]);
  ASSERT_TRUE(test);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
