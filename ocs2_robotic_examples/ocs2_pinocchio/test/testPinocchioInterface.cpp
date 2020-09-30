#include <gtest/gtest.h>

#include <ocs2_pinocchio/PinocchioInterface.h>
#include "CartPoleUrdf.h"

TEST(testPinocchioInterface, buildFromXml) {
  auto pinocchio = ocs2::PinocchioInterface<ocs2::scalar_t>::buildFromXml(cartPoleUrdf);
  std::cout << pinocchio;
}
