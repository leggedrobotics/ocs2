#include <gtest/gtest.h>

#include <ocs2_pinocchio_interface/urdf.h>

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_switched_model_interface/analytical_inverse_kinematics/AnalyticalInverseKinematics.h>

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/QuadrupedKinematics.h>
#include <ocs2_anymal_models/package_path.h>

#include <iostream>
#include <string>

using namespace anymal;

class QuadrupedInverseKinematicsTest : public ::testing::Test {
 public:
  QuadrupedInverseKinematicsTest()
      : frameDeclaration(frameDeclarationFromFile(getPath() + "/urdf/frame_declaration_anymal_c.info")),
        pinocchioInterface(ocs2::getPinocchioInterfaceFromUrdfString(getUrdfString(AnymalModel::Camel))),
        kinematics_(frameDeclaration, pinocchioInterface) {
    srand(10);

    auto& data = pinocchioInterface.getData();
    const auto& model = pinocchioInterface.getModel();

    switched_model::joint_coordinate_t zeroConfiguration(switched_model::joint_coordinate_t::Zero());
    pinocchio::forwardKinematics(model, data, zeroConfiguration);
    pinocchio::updateFramePlacements(model, data);

    for (size_t leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
      const auto& hipTransform = data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[0], pinocchioInterface)];
      const auto& thighTransform = data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[1], pinocchioInterface)];
      const auto& shankTransform = data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].joints[2], pinocchioInterface)];
      const auto& footTransform = data.oMf[QuadrupedPinocchioMapping::getBodyId(frameDeclaration.legs[leg].tip, pinocchioInterface)];

      legParametersPerLeg[leg] = switched_model::analytical_inverse_kinematics::LegInverseKinematicParameters(
          hipTransform.translation(), thighTransform.translation() - hipTransform.translation(),
          shankTransform.translation() - thighTransform.translation(), footTransform.translation() - shankTransform.translation());
    }
  }

  FrameDeclaration frameDeclaration;
  ocs2::PinocchioInterface pinocchioInterface;
  QuadrupedKinematics kinematics_;
  switched_model::feet_array_t<switched_model::analytical_inverse_kinematics::LegInverseKinematicParameters> legParametersPerLeg;
};

TEST_F(QuadrupedInverseKinematicsTest, defaultJointAngles) {
  switched_model::joint_coordinate_t jointPositions;
  jointPositions << -0.25, 0.60, -0.85, 0.25, 0.60, -0.85, -0.25, -0.60, 0.85, 0.25, -0.60, 0.85;
  auto jointsPerLeg = switched_model::toArray(jointPositions);

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    switched_model::vector3_t positionBaseToFootInBaseFrame = kinematics_.positionBaseToFootInBaseFrame(footIndex, jointPositions);

    switched_model::vector3_t legJoints;
    getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, positionBaseToFootInBaseFrame, legParametersPerLeg[footIndex],
                                                           footIndex);
    EXPECT_TRUE(jointsPerLeg[footIndex].isApprox(legJoints));
  }
}

TEST_F(QuadrupedInverseKinematicsTest, randomJointAngles) {
  switched_model::joint_coordinate_t jointPositionsDefault;
  jointPositionsDefault << -0.25, 0.60, -0.85, 0.25, 0.60, -0.85, -0.25, -0.60, 0.85, 0.25, -0.60, 0.85;
  const auto jointsPerLegDefault = switched_model::toArray(jointPositionsDefault);
  const switched_model::scalar_t perturbation = 0.4;
  const int Ntest = 100;

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    for (int i = 0; i < Ntest; ++i) {
      auto jointsPerLeg = jointsPerLegDefault;
      jointsPerLeg[footIndex] += perturbation * switched_model::vector3_t::Random();

      // Forward kinematics
      switched_model::vector3_t positionBaseToFootInBaseFrame =
          kinematics_.positionBaseToFootInBaseFrame(footIndex, switched_model::fromArray(jointsPerLeg));

      // Inverse kinematics
      switched_model::vector3_t legJoints;
      getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, positionBaseToFootInBaseFrame, legParametersPerLeg[footIndex],
                                                             footIndex);

      EXPECT_TRUE(jointsPerLeg[footIndex].isApprox(legJoints))
          << "Foot: " << footIndex << ", True angles: " << jointsPerLeg[footIndex].transpose() << ", IK: " << legJoints.transpose();
    }
  }
}

TEST_F(QuadrupedInverseKinematicsTest, outOfRange) {
  const int Ntest = 100;

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    for (int i = 0; i < Ntest; ++i) {
      const switched_model::vector3_t outOfRangeTarget = 1000.0 * switched_model::vector3_t::Random();

      // Inverse kinematics
      switched_model::vector3_t legJoints;
      getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, outOfRangeTarget, legParametersPerLeg[footIndex], footIndex);

      // Forward kinematics
      switched_model::feet_array_t<switched_model::vector3_t> jointsPerLeg =
          switched_model::constantFeetArray<switched_model::vector3_t>(switched_model::vector3_t::Zero());
      jointsPerLeg[footIndex] = legJoints;

      switched_model::vector3_t positionBaseToFootInBaseFrame =
          kinematics_.positionBaseToFootInBaseFrame(footIndex, switched_model::fromArray(jointsPerLeg));

      // Inverse kinematics
      getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, outOfRangeTarget, legParametersPerLeg[footIndex], footIndex);

      EXPECT_TRUE(jointsPerLeg[footIndex].allFinite())
          << "Foot: " << footIndex << " First IK not finite! " << jointsPerLeg[footIndex].transpose();
      EXPECT_TRUE(legJoints.allFinite()) << "Foot: " << footIndex << " Second IK not finite! " << legJoints.transpose();
      EXPECT_TRUE(jointsPerLeg[footIndex].isApprox(legJoints))
          << "Foot: " << footIndex << ", First IK: " << jointsPerLeg[footIndex].transpose() << ", second IK: " << legJoints.transpose();
    }
  }
}

TEST_F(QuadrupedInverseKinematicsTest, closeToHip) {
  const int Ntest = 100;

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    for (int i = 0; i < Ntest; ++i) {
      switched_model::vector3_t outOfRangeTarget = legParametersPerLeg[footIndex].positionBaseToHaaCenterInBaseFrame_;
      if (i > 0) {  // test equal to HAA center for the first test, afterward test in range close to the hip
        outOfRangeTarget += legParametersPerLeg[footIndex].positionHipToFootYoffset_ * switched_model::vector3_t::Random();
      }

      // Inverse kinematics
      switched_model::vector3_t legJoints;
      getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, outOfRangeTarget, legParametersPerLeg[footIndex], footIndex);

      // Forward kinematics
      switched_model::feet_array_t<switched_model::vector3_t> jointsPerLeg =
          switched_model::constantFeetArray<switched_model::vector3_t>(switched_model::vector3_t::Zero());
      jointsPerLeg[footIndex] = legJoints;

      switched_model::vector3_t positionBaseToFootInBaseFrame =
          kinematics_.positionBaseToFootInBaseFrame(footIndex, switched_model::fromArray(jointsPerLeg));

      // Inverse kinematics
      getLimbJointPositionsFromPositionBaseToFootInBaseFrame(legJoints, outOfRangeTarget, legParametersPerLeg[footIndex], footIndex);

      EXPECT_TRUE(jointsPerLeg[footIndex].allFinite())
          << "Foot: " << footIndex << " First IK not finite! " << jointsPerLeg[footIndex].transpose();
      EXPECT_TRUE(legJoints.allFinite()) << "Foot: " << footIndex << " Second IK not finite! " << legJoints.transpose();
      EXPECT_TRUE(jointsPerLeg[footIndex].isApprox(legJoints))
          << "Foot: " << footIndex << ", First IK: " << jointsPerLeg[footIndex].transpose() << ", second IK: " << legJoints.transpose();
    }
  }
}
