#include <gtest/gtest.h>

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/camel/AnymalCamelCom.h>

using namespace anymal;

class QuadrupedComTest : public ::testing::Test {
 public:
  QuadrupedComTest()
      : pinocchioCom_(createQuadrupedPinocchioInterfaceFromUrdfString(getUrdfString(AnymalModel::Camel)), QuadrupedMapping({0, 2, 1, 3})) {
    srand(0);
  }

  QuadrupedCom pinocchioCom_;
  AnymalCamelCom robcogenCom_;
};

TEST_F(QuadrupedComTest, totalMass) {
  EXPECT_FLOAT_EQ(pinocchioCom_.totalMass(), robcogenCom_.totalMass());
}

TEST_F(QuadrupedComTest, calculateBaseLocalAccelerations) {
  const ocs2::vector_t basePose = switched_model::base_coordinate_t::Random();
  const ocs2::vector_t baseLocalVelocities = switched_model::base_coordinate_t::Random();

  const ocs2::vector_t jointPositions = switched_model::joint_coordinate_t::Random();
  const ocs2::vector_t jointVelocities = switched_model::joint_coordinate_t::Random();
  const ocs2::vector_t jointAccelerations = switched_model::joint_coordinate_t::Random();
  const ocs2::vector_t forcesOnBaseInBaseFrame = switched_model::base_coordinate_t::Random();

  ocs2::vector_t rAcc = robcogenCom_.calculateBaseLocalAccelerations(basePose, baseLocalVelocities, jointPositions, jointVelocities,
                                                                     jointAccelerations, forcesOnBaseInBaseFrame);
  ocs2::vector_t pAcc = pinocchioCom_.calculateBaseLocalAccelerations(basePose, baseLocalVelocities, jointPositions, jointVelocities,
                                                                      jointAccelerations, forcesOnBaseInBaseFrame);

  EXPECT_TRUE(pAcc.isApprox(rAcc, 1e-8)) << "Pinocchio: \n" << pAcc.transpose() << "\nRoboGen: \n" << rAcc.transpose();
}
