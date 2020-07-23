#include <gtest/gtest.h>

#include <ocs2_core/control/FeedforwardController.h>

using namespace ocs2;

TEST(testFeedforwardController, testSerialization) {
  scalar_array_t time = {0.0, 1.0};
  vector_array_t uff = {vector_t::Random(2), vector_t::Random(2)};
  FeedforwardController controller(time, uff);

  std::vector<std::vector<float>> data(2);
  std::vector<std::vector<float>*> dataPtr{&data[0], &data[1]};
  std::vector<std::vector<float> const*> dataPtrConst{&data[0], &data[1]};

  controller.flatten(time, dataPtr);

  auto controllerOut = FeedforwardController::unFlatten(time, dataPtrConst);

  for (int k = 0; k < time.size(); k++) {
    EXPECT_NEAR(controller.timeStamp_[k], controllerOut.timeStamp_[k], 1e-6);
    EXPECT_TRUE(controller.uffArray_[k].isApprox(controllerOut.uffArray_[k], 1e-6));
  }
}
