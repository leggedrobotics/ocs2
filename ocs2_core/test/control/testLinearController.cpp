#include <gtest/gtest.h>

#include <ocs2_core/control/LinearController.h>

using namespace ocs2;

TEST(testLinearController, testSerialization) {
  scalar_array_t time = {0.0, 1.0};
  vector_array_t bias = {vector_t::Random(2), vector_t::Random(2)};
  matrix_array_t gain = {matrix_t::Random(2, 3), matrix_t::Random(2, 3)};
  LinearController controller(time, bias, gain);

  std::vector<std::vector<float>> data(2);
  std::vector<std::vector<float>*> dataPtr{&data[0], &data[1]};
  std::vector<std::vector<float> const*> dataPtrConst{&data[0], &data[1]};

  controller.flatten(time, dataPtr);

  auto controllerOut = LinearController::unFlatten({3, 3}, {2, 2}, time, dataPtrConst);

  for (int k = 0; k < time.size(); k++) {
    EXPECT_NEAR(controller.timeStamp_[k], controllerOut.timeStamp_[k], 1e-6);
    EXPECT_TRUE(controller.gainArray_[k].isApprox(controllerOut.gainArray_[k], 1e-6));
    EXPECT_TRUE(controller.biasArray_[k].isApprox(controllerOut.biasArray_[k], 1e-6));
  }
}
