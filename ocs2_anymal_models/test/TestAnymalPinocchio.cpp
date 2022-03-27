#include <gtest/gtest.h>

#include <ocs2_anymal_models/AnymalKinematics.h>
#include <ocs2_anymal_models/camel/AnymalCamelKinematics.h>

#include <iostream>

using namespace anymal;

class AnymalPinocchioTest : public ::testing::Test {
 public:
  AnymalPinocchioTest()
      : anymalPinocchio_(
            "/home/boom/anymal_ws/src/anymal_rsl/anymal_c_rsl/anymal_camel_rsl/anymal_camel_rsl/urdf/cached_anymal_camel_rsl.urdf") {
    srand(10);
  }

  AnymalKinematics anymalPinocchio_;
  AnymalCamelKinematics anymalCamel;
};

TEST_F(AnymalPinocchioTest, baseToLegRootInBaseFrame) {
  for (int i = 0; i < 4; i++) {
    switched_model::vector3_t pinocchio = anymalPinocchio_.baseToLegRootInBaseFrame(i);
    switched_model::vector3_t robcogen = anymalCamel.baseToLegRootInBaseFrame(i);
    EXPECT_TRUE(pinocchio.isApprox(robcogen)) << "i: " << i << "\nPinocchio: " << pinocchio.transpose()
                                              << "\n\n RobCoGen: " << robcogen.transpose();
  }
}

TEST_F(AnymalPinocchioTest, baseToFootJacobianBlockInBaseFrame) {
  switched_model::joint_coordinate_t q;
  q.setRandom();

  for (int i = 0; i < 4; i++) {
    ocs2::matrix_t pinocchio = anymalPinocchio_.baseToFootJacobianBlockInBaseFrame(i, q);
    ocs2::matrix_t robcogen = anymalCamel.baseToFootJacobianBlockInBaseFrame(i, q);
    EXPECT_TRUE(pinocchio.isApprox(robcogen)) << "i: " << i << "\nPinocchio:\n" << pinocchio << "\n\n RobCoGen:\n" << robcogen;
  }
}

TEST_F(AnymalPinocchioTest, footOrientationInBaseFrame) {
  switched_model::joint_coordinate_t q;
  q.setRandom();

  for (int i = 0; i < 4; i++) {
    ocs2::matrix_t pinocchio = anymalPinocchio_.footOrientationInBaseFrame(i, q);
    ocs2::matrix_t robcogen = anymalCamel.footOrientationInBaseFrame(i, q);
    EXPECT_TRUE(pinocchio.isApprox(robcogen)) << "i: " << i << "\nPinocchio:\n" << pinocchio << "\n\n RobCoGen:\n" << robcogen;
  }
}

TEST_F(AnymalPinocchioTest, collisionSpheresInBaseFrame) {
  switched_model::joint_coordinate_t q;
  q.setRandom();
  // q << 0, 1.5, 0, 0, 1.53, 0, 0, 0, 0, 0, 1.59;

  auto pinocchio = anymalPinocchio_.collisionSpheresInBaseFrame(q);
  auto robcogen = anymalCamel.collisionSpheresInBaseFrame(q);
  for (std::size_t i = 0; i < switched_model::NUM_CONTACT_POINTS; i++) {
    EXPECT_TRUE(pinocchio[i].position.isApprox(robcogen[i].position)) << "i: " << i << "\nPinocchio:\n"
                                                                      << pinocchio[i].position.transpose() << "\nRobCoGen:\n"
                                                                      << robcogen[i].position.transpose() << "\n";
  }
}
