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
#include <ocs2_anymal_models/QuadrupedInverseKinematics.h>
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
        kinematics_(frameDeclaration, pinocchioInterface),
        inverseKinematics_(frameDeclaration, pinocchioInterface) {
    srand(10);
  }

  FrameDeclaration frameDeclaration;
  ocs2::PinocchioInterface pinocchioInterface;
  QuadrupedKinematics kinematics_;
  QuadrupedInverseKinematics inverseKinematics_;
  switched_model::feet_array_t<switched_model::analytical_inverse_kinematics::LegInverseKinematicParameters> legParametersPerLeg;
};

TEST_F(QuadrupedInverseKinematicsTest, defaultJointAngles) {
  switched_model::joint_coordinate_t jointPositions;
  jointPositions << -0.25, 0.60, -0.85, 0.25, 0.60, -0.85, -0.25, -0.60, 0.85, 0.25, -0.60, 0.85;
  auto jointsPerLeg = switched_model::toArray(jointPositions);

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    switched_model::vector3_t positionBaseToFootInBaseFrame = kinematics_.positionBaseToFootInBaseFrame(footIndex, jointPositions);

    switched_model::vector3_t legJoints =
        inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footIndex, positionBaseToFootInBaseFrame);
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
      switched_model::vector3_t legJoints =
          inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footIndex, positionBaseToFootInBaseFrame);

      EXPECT_TRUE(jointsPerLeg[footIndex].isApprox(legJoints))
          << "Foot: " << footIndex << ", True angles: " << jointsPerLeg[footIndex].transpose() << ", IK: " << legJoints.transpose();
    }
  }
}

TEST_F(QuadrupedInverseKinematicsTest, outOfRange) {
  const int Ntest = 100;
  const double tol = 1e-6;

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    for (int i = 0; i < Ntest; ++i) {
      const switched_model::vector3_t outOfRangeTarget = 1000.0 * switched_model::vector3_t::Random();

      // Inverse kinematics
      switched_model::vector3_t legJoints =
          inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footIndex, outOfRangeTarget);

      // Forward kinematics
      switched_model::feet_array_t<switched_model::vector3_t> jointsPerLeg =
          switched_model::constantFeetArray<switched_model::vector3_t>(switched_model::vector3_t::Zero());
      jointsPerLeg[footIndex] = legJoints;

      switched_model::vector3_t positionBaseToFootInBaseFrame =
          kinematics_.positionBaseToFootInBaseFrame(footIndex, switched_model::fromArray(jointsPerLeg));

      // Inverse kinematics
      legJoints = inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footIndex, positionBaseToFootInBaseFrame);

      EXPECT_TRUE(jointsPerLeg[footIndex].allFinite())
          << "Foot: " << footIndex << " First IK not finite! " << jointsPerLeg[footIndex].transpose();
      EXPECT_TRUE(legJoints.allFinite()) << "Foot: " << footIndex << " Second IK not finite! " << legJoints.transpose();
      EXPECT_TRUE(jointsPerLeg[footIndex].isApprox(legJoints, tol))
          << "Foot: " << footIndex << ", First IK: " << jointsPerLeg[footIndex].transpose() << ", second IK: " << legJoints.transpose();
    }
  }
}

TEST_F(QuadrupedInverseKinematicsTest, closeToHip) {
  const int Ntest = 100;
  const double tol = 1e-6;

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    for (int i = 0; i < Ntest; ++i) {
      switched_model::vector3_t outOfRangeTarget = legParametersPerLeg[footIndex].positionBaseToHaaCenterInBaseFrame_;
      if (i > 0) {  // test equal to HAA center for the first test, afterward test in range close to the hip
        outOfRangeTarget += legParametersPerLeg[footIndex].positionHipToFootYoffset_ * switched_model::vector3_t::Random();
      }

      // Inverse kinematics
      switched_model::vector3_t legJoints =
          inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footIndex, outOfRangeTarget);

      // Forward kinematics
      switched_model::feet_array_t<switched_model::vector3_t> jointsPerLeg =
          switched_model::constantFeetArray<switched_model::vector3_t>(switched_model::vector3_t::Zero());
      jointsPerLeg[footIndex] = legJoints;

      switched_model::vector3_t positionBaseToFootInBaseFrame =
          kinematics_.positionBaseToFootInBaseFrame(footIndex, switched_model::fromArray(jointsPerLeg));

      // Inverse kinematics
      legJoints = inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footIndex, positionBaseToFootInBaseFrame);

      EXPECT_TRUE(jointsPerLeg[footIndex].allFinite())
          << "Foot: " << footIndex << " First IK not finite! " << jointsPerLeg[footIndex].transpose();
      EXPECT_TRUE(legJoints.allFinite()) << "Foot: " << footIndex << " Second IK not finite! " << legJoints.transpose();
      EXPECT_TRUE(jointsPerLeg[footIndex].isApprox(legJoints, tol))
          << "Foot: " << footIndex << ", First IK: " << jointsPerLeg[footIndex].transpose() << ", second IK: " << legJoints.transpose();
    }
  }
}

TEST_F(QuadrupedInverseKinematicsTest, defaultJointAnglesVelocity) {
  const double tol = 1e-6;
  const double damping = 1e-9;

  switched_model::joint_coordinate_t jointPositions;
  jointPositions << -0.25, 0.60, -0.85, 0.25, 0.60, -0.85, -0.25, -0.60, 0.85, 0.25, -0.60, 0.85;

  switched_model::joint_coordinate_t jointVelocities;
  jointVelocities.setRandom();
  auto jointVelocitiesPerLeg = switched_model::toArray(jointVelocities);

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    switched_model::vector3_t footVelocity = kinematics_.footVelocityRelativeToBaseInBaseFrame(footIndex, jointPositions, jointVelocities);

    switched_model::vector3_t legJointVelocities = inverseKinematics_.getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
        footIndex, footVelocity, kinematics_.baseToFootJacobianBlockInBaseFrame(footIndex, jointPositions), damping);
    EXPECT_TRUE(jointVelocitiesPerLeg[footIndex].isApprox(legJointVelocities, tol));
  }
}

TEST_F(QuadrupedInverseKinematicsTest, singularityJointAnglesVelocity) {
  const int Ntest = 100;

  const double tol = 1e-6;
  const double damping = 1e-9;

  switched_model::joint_coordinate_t jointVelocities;
  jointVelocities.setRandom();
  auto jointVelocitiesPerLeg = switched_model::toArray(jointVelocities);

  for (size_t footIndex = 0; footIndex < switched_model::NUM_CONTACT_POINTS; ++footIndex) {
    for (int i = 0; i < Ntest; ++i) {
      // Inverse kinematics to create singular configuration
      const switched_model::vector3_t outOfRangeTarget = 1000.0 * switched_model::vector3_t::Random();
      switched_model::vector3_t legJoints =
          inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footIndex, outOfRangeTarget);

      // Set joint positions
      switched_model::feet_array_t<switched_model::vector3_t> jointsPerLeg =
          switched_model::constantFeetArray<switched_model::vector3_t>(switched_model::vector3_t::Zero());
      jointsPerLeg[footIndex] = legJoints;
      auto jointPositions = switched_model::fromArray(jointsPerLeg);

      switched_model::vector3_t footVelocity =
          kinematics_.footVelocityRelativeToBaseInBaseFrame(footIndex, jointPositions, jointVelocities);

      auto J = kinematics_.baseToFootJacobianBlockInBaseFrame(footIndex, jointPositions);

      switched_model::vector3_t legJointVelocities =
          inverseKinematics_.getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(footIndex, footVelocity, J, damping);
      EXPECT_TRUE(legJointVelocities.allFinite());

      // Compare on the achieved foot velocity
      EXPECT_TRUE(footVelocity.isApprox(J.block<3, 3>(3, 0) * legJointVelocities, tol))
          << "Foot: " << footIndex << ", Velocities: " << jointVelocitiesPerLeg[footIndex].transpose()
          << ", IK: " << legJointVelocities.transpose();
    }
  }
}
