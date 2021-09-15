//
// Created by rgrandia on 25.09.20.
//

#include "TestAnymalSwitchedModel.h"

#include <ocs2_anymal_models/AnymalModels.h>

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include <ocs2_anymal_models/chip/generated/inverse_dynamics.h>
#include "ocs2_anymal_models/chip/generated/forward_dynamics.h"
#include "ocs2_anymal_models/chip/generated/inertia_properties.h"
#include "ocs2_anymal_models/chip/generated/jsim.h"
#include "ocs2_anymal_models/chip/generated/transforms.h"

using namespace anymal;

class AnymalChipSwitchedModelTests : public switched_model::TestAnymalSwitchedModel {
 public:
  AnymalChipSwitchedModelTests()
      : TestAnymalSwitchedModel(getAnymalKinematics(AnymalModel::Chip), getAnymalKinematicsAd(AnymalModel::Chip),
                                getAnymalComModel(AnymalModel::Chip), getAnymalComModelAd(AnymalModel::Chip),
                                getWholebodyDynamics(AnymalModel::Chip)) {}
};

TEST_F(AnymalChipSwitchedModelTests, Cost) {
  this->testCosts();
}

TEST_F(AnymalChipSwitchedModelTests, Constraints) {
  this->testConstraints();
}

TEST_F(AnymalChipSwitchedModelTests, Kinematics) {
  this->printKinematics();
}

TEST_F(AnymalChipSwitchedModelTests, ComDynamics) {
  this->printComModel();
}

TEST_F(AnymalChipSwitchedModelTests, baseDynamics) {
  this->testBaseDynamics();
}

TEST_F(AnymalChipSwitchedModelTests, fullDynamics) {
  const switched_model::rbd_state_t rbdState = switched_model::rbd_state_t::Random();
  const switched_model::base_coordinate_t forcesOnBaseInBaseFrame = switched_model::base_coordinate_t::Random();
  const switched_model::joint_coordinate_t tau = switched_model::joint_coordinate_t::Random();
  const switched_model::base_coordinate_t qBase = switched_model::getBasePose(rbdState);
  const switched_model::base_coordinate_t qdBase = switched_model::getBaseLocalVelocity(rbdState);
  const switched_model::joint_coordinate_t qj = switched_model::getJointPositions(rbdState);
  const switched_model::joint_coordinate_t dqj = switched_model::getJointVelocities(rbdState);

  switched_model::vector6_t gravity;
  const auto b_R_o = switched_model::rotationMatrixOriginToBase(switched_model::getOrientation(switched_model::getBasePose(rbdState)));
  gravity << switched_model::vector3_t::Zero(), b_R_o * switched_model::vector3_t(0.0, 0.0, -9.81);

  using trait_t = typename iit::rbd::tpl::TraitSelector<double>::Trait;
  iit::chip::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::chip::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::chip::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::chip::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::chip::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);
  iit::chip::dyn::tpl::ForwardDynamics<trait_t> forwardDynamics_(inertiaProperties_, motionTransforms_);

  iit::chip::dyn::tpl::ForwardDynamics<trait_t>::JointState jointAccelerationFd;
  iit::chip::dyn::tpl::ForwardDynamics<trait_t>::Acceleration baseAccelerationFd;
  iit::chip::dyn::tpl::ForwardDynamics<trait_t>::ExtForces extForces(switched_model::base_coordinate_t::Zero());
  extForces[iit::chip::LinkIdentifiers::BASE] = forcesOnBaseInBaseFrame;
  forwardDynamics_.fd(jointAccelerationFd, baseAccelerationFd, qdBase, gravity, qj, dqj, tau, extForces);

  const auto baseAcceleration =
      this->comModel_->calculateBaseLocalAccelerations(qBase, qdBase, qj, dqj, jointAccelerationFd, forcesOnBaseInBaseFrame);

  ASSERT_TRUE(baseAcceleration.isApprox(baseAccelerationFd));
}

TEST_F(AnymalChipSwitchedModelTests, EndeffectorOrientation) {
  this->testEndeffectorOrientation();
}

TEST_F(AnymalChipSwitchedModelTests, EndeffectorAlignedYAxisRandomHFEKFE) {
  this->testEndeffectorAlignedYAxisRandomHFEKFE();
}

TEST_F(AnymalChipSwitchedModelTests, EndeffectorAlignedXAxisRandomHAA) {
  this->testEndeffectorAlignedXAxisRandomHAA();
}