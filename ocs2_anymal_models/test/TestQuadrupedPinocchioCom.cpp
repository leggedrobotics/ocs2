#include <gtest/gtest.h>

#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_switched_model_interface/core/Rotations.h>

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/QuadrupedInverseKinematics.h>
#include <ocs2_anymal_models/QuadrupedKinematics.h>
#include <ocs2_anymal_models/package_path.h>

#include "camel/AnymalCamelCom.h"

using namespace anymal;

class QuadrupedComTest : public ::testing::Test {
 public:
  QuadrupedComTest()
      : frameDeclaration(frameDeclarationFromFile(getPath() + "/urdf/frame_declaration_anymal_c.info")),
        pinocchioInterface(createQuadrupedPinocchioInterfaceFromUrdfString(getUrdfString(AnymalModel::Camel))),
        pinocchioCom_(frameDeclaration, pinocchioInterface),
        kinematics_(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(getUrdfString(AnymalModel::Camel))),
        inverseKinematics_(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(getUrdfString(AnymalModel::Camel))) {
    srand(0);
  }

  FrameDeclaration frameDeclaration;
  ocs2::PinocchioInterface pinocchioInterface;
  QuadrupedCom pinocchioCom_;
  AnymalCamelCom robcogenCom_;
  QuadrupedKinematics kinematics_;
  QuadrupedInverseKinematics inverseKinematics_;
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

TEST_F(QuadrupedComTest, comWorldAndBase) {
  const switched_model::base_coordinate_t basePose = switched_model::base_coordinate_t::Random();
  const switched_model::joint_coordinate_t jointPositions = switched_model::joint_coordinate_t::Random();

  const auto comInWorld = pinocchioCom_.centerOfMassInWorldFrame(basePose, jointPositions);
  const auto comInBase = pinocchioCom_.centerOfMassInBaseFrame(jointPositions);
  const switched_model::vector3_t comInWorldCheck =
      switched_model::rotateVectorBaseToOrigin(comInBase, switched_model::getOrientation(basePose)) +
      switched_model::getPositionInOrigin(basePose);

  ASSERT_TRUE(comInWorld.isApprox(comInWorldCheck));
}

TEST_F(QuadrupedComTest, comInBasePerConfiguration) {
  switched_model::base_coordinate_t basePose = switched_model::base_coordinate_t::Zero();
  switched_model::joint_coordinate_t jointPositionsDefault;
  jointPositionsDefault << -0.25, 0.60, -0.85, 0.25, 0.60, -0.85, -0.25, -0.60, 0.85, 0.25, -0.60, 0.85;

  const auto baseToFeetDefault = kinematics_.positionBaseToFeetInBaseFrame(jointPositionsDefault);
  auto hipToFeetDefault = baseToFeetDefault;
  for (int leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    hipToFeetDefault[leg] -= kinematics_.baseToLegRootInBaseFrame(leg);
  }

  for (ocs2::scalar_t pitchDeg = -50; pitchDeg < 51.0; pitchDeg += 5.0) {
    ocs2::scalar_t pitch = pitchDeg * (M_PI / 180.0);
    basePose[1] = pitch;

    const auto comInWorldDefault = pinocchioCom_.centerOfMassInWorldFrame(basePose, jointPositionsDefault);
    const auto comInBaseDefault = pinocchioCom_.centerOfMassInBaseFrame(jointPositionsDefault);

    // Adaptive legs
    switched_model::joint_coordinate_t jointPositionsAdaptive;
    for (int leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
      switched_model::vector3_t baseToFootInBase =
          kinematics_.baseToLegRootInBaseFrame(leg) +
          switched_model::rotateVectorOriginToBase(hipToFeetDefault[leg], switched_model::getOrientation(basePose));
      jointPositionsAdaptive.segment<3>(3 * leg) =
          inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(leg, baseToFootInBase);
    }

    const auto comInWorldAdaptive = pinocchioCom_.centerOfMassInWorldFrame(basePose, jointPositionsAdaptive);
    const auto comInBaseAdaptive = pinocchioCom_.centerOfMassInBaseFrame(jointPositionsAdaptive);

    //    std::cout << "Pitch: " << pitchDeg << ", CoM default: " << comInWorldDefault.transpose()
    //              << ", CoM adaptive: " << comInWorldAdaptive.transpose() << std::endl;
    std::cout << pitchDeg << ", " << comInWorldDefault.x() << ", " << comInWorldAdaptive.x() << std::endl;
  }
}