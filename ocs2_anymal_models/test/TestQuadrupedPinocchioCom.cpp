#include <gtest/gtest.h>

#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/camel/AnymalCamelCom.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ros/package.h>

#include <iostream>
#include <string>

using namespace anymal;

class QuadrupedComTest : public ::testing::Test {
 public:
  QuadrupedComTest()
      : pinocchioCom_(createQuadrupedPinocchioInterface(ros::package::getPath("anymal_camel_rsl") + "/urdf/cached_anymal_camel_rsl.urdf")) {
    srand(10);
  }

  QuadrupedCom pinocchioCom_;
  AnymalCamelCom robcogenCom_;
};

TEST_F(QuadrupedComTest, totalMass) {
  EXPECT_FLOAT_EQ(pinocchioCom_.totalMass(), robcogenCom_.totalMass());
}

TEST_F(QuadrupedComTest, calculateBaseLocalAccelerations) {
  const ocs2::vector_t basePose = switched_model::base_coordinate_t::Zero();
  const ocs2::vector_t baseLocalVelocities =
      (switched_model::base_coordinate_t() << ocs2::vector_t::Random(3), ocs2::vector_t::Random(3)).finished();

  const ocs2::vector_t jointPositions = switched_model::joint_coordinate_t::Zero();
  const ocs2::vector_t jointVelocities = switched_model::joint_coordinate_t::Zero();
  const ocs2::vector_t jointAccelerations = switched_model::joint_coordinate_t::Zero();
  const ocs2::vector_t forcesOnBaseInBaseFrame = switched_model::base_coordinate_t::Zero();

  ocs2::vector_t rAcc = robcogenCom_.calculateBaseLocalAccelerations(basePose, baseLocalVelocities, jointPositions, jointVelocities,
                                                                     jointAccelerations, forcesOnBaseInBaseFrame);
  ocs2::vector_t pAcc = pinocchioCom_.calculateBaseLocalAccelerations(basePose, baseLocalVelocities, jointPositions, jointVelocities,
                                                                      jointAccelerations, forcesOnBaseInBaseFrame);

  EXPECT_TRUE(pAcc.isApprox(rAcc)) << "Pinocchio: " << pAcc.transpose() << "\nRoboGen: " << rAcc.transpose();
}

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

TEST(Link, testLink) {
  auto p = createQuadrupedPinocchioInterface(ros::package::getPath("ocs2_anymal_models") + "/test/link.urdf");
  auto& data = p.getData();
  const auto& model = p.getModel();
  ocs2::vector_t velocity(6);
  // velocity << ocs2::vector_t::Random(6);
  velocity << 10, 0, 10, 0, 0, 0;

  const Eigen::Quaterniond baseQuat(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(-90.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

  ocs2::vector_t configuration(7);
  configuration << baseQuat.coeffs(), 0, 0, 0;
  ocs2::vector_t c = pinocchio::nonLinearEffects(model, data, configuration, velocity);
  ocs2::vector_t g = pinocchio::computeGeneralizedGravity(model, data, configuration);
  std::cerr << "c: " << c.transpose() << "\n"
            << "g: " << g.transpose() << "\n";
}