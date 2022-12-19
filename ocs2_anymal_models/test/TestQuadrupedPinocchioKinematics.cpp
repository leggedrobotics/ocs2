#include <gtest/gtest.h>

#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_anymal_models/QuadrupedKinematics.h>
#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/package_path.h>

#include "camel/AnymalCamelKinematics.h"

#include <iostream>
#include <string>

using namespace anymal;

class QuadrupedKinematicsTest : public ::testing::Test {
 public:
  QuadrupedKinematicsTest()
      : frameDeclaration(frameDeclarationFromFile(getPath() + "/urdf/frame_declaration_anymal_c.info")),
        pinocchioInterface(ocs2::getPinocchioInterfaceFromUrdfString(getUrdfString(AnymalModel::Camel))),
        pinocchioKinematics_(frameDeclaration, pinocchioInterface) {
    srand(10);
  }

  FrameDeclaration frameDeclaration;
  ocs2::PinocchioInterface pinocchioInterface;
  QuadrupedKinematics pinocchioKinematics_;
  AnymalCamelKinematics robcogenKinematics_;
};

TEST_F(QuadrupedKinematicsTest, baseToLegRootInBaseFrame) {
  for (int i = 0; i < 4; i++) {
    switched_model::vector3_t pinocchio = pinocchioKinematics_.baseToLegRootInBaseFrame(i);
    switched_model::vector3_t robcogen = robcogenKinematics_.baseToLegRootInBaseFrame(i);
    EXPECT_TRUE(pinocchio.isApprox(robcogen)) << "i: " << i << "\nPinocchio: " << pinocchio.transpose()
                                              << "\n\n RobCoGen: " << robcogen.transpose();
  }
}

TEST_F(QuadrupedKinematicsTest, baseToFootJacobianBlockInBaseFrame) {
  switched_model::joint_coordinate_t q;
  q.setRandom();

  for (int i = 0; i < 4; i++) {
    ocs2::matrix_t pinocchio = pinocchioKinematics_.baseToFootJacobianBlockInBaseFrame(i, q);
    ocs2::matrix_t robcogen = robcogenKinematics_.baseToFootJacobianBlockInBaseFrame(i, q);
    EXPECT_TRUE(pinocchio.isApprox(robcogen)) << "i: " << i << "\nPinocchio:\n" << pinocchio << "\n\n RobCoGen:\n" << robcogen;
  }
}

TEST_F(QuadrupedKinematicsTest, footOrientationInBaseFrame) {
  switched_model::joint_coordinate_t q;
  q.setRandom();

  for (int i = 0; i < 4; i++) {
    ocs2::matrix_t pinocchio = pinocchioKinematics_.footOrientationInBaseFrame(i, q);
    ocs2::matrix_t robcogen = robcogenKinematics_.footOrientationInBaseFrame(i, q);
    EXPECT_TRUE(pinocchio.isApprox(robcogen)) << "i: " << i << "\nPinocchio:\n" << pinocchio << "\n\n RobCoGen:\n" << robcogen;
  }
}

TEST_F(QuadrupedKinematicsTest, collisionSpheresInBaseFrame) {
  switched_model::joint_coordinate_t q;
  q.setRandom();
  // q << 0, 1.5, 0, 0, 1.53, 0, 0, 0, 0, 0, 1.59;

  auto pinocchio = pinocchioKinematics_.collisionSpheresInBaseFrame(q);
  auto robcogen = robcogenKinematics_.collisionSpheresInBaseFrame(q);
  for (std::size_t i = 0; i < switched_model::NUM_CONTACT_POINTS; i++) {
    EXPECT_TRUE(pinocchio[i].position.isApprox(robcogen[i].position)) << "i: " << i << "\nPinocchio:\n"
                                                                      << pinocchio[i].position.transpose() << "\nRobCoGen:\n"
                                                                      << robcogen[i].position.transpose() << "\n";
  }
}

TEST_F(QuadrupedKinematicsTest, footVelocityRelativeToBaseInBaseFrame) {
  switched_model::joint_coordinate_t q;
  q.setRandom();
  switched_model::joint_coordinate_t v;
  v.setRandom();

  for (std::size_t i = 0; i < switched_model::NUM_CONTACT_POINTS; i++) {
    auto pinocchio = pinocchioKinematics_.footVelocityRelativeToBaseInBaseFrame(i, q, v);
    auto reference = robcogenKinematics_.footVelocityRelativeToBaseInBaseFrame(i, q, v);
    EXPECT_TRUE(pinocchio.isApprox(reference)) << "i: " << i << "\nPinocchio:\n"
                                               << pinocchio.transpose() << "\nReference:\n"
                                               << reference.transpose() << "\n";
  }
}
