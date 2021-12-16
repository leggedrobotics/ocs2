//
// Created by rgrandia on 25.09.20.
//

#include "TestAnymalSwitchedModel.h"

#include <ocs2_anymal_models/AnymalModels.h>

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include <ocs2_anymal_models/camel/generated/inverse_dynamics.h>
#include "ocs2_anymal_models/camel/generated/forward_dynamics.h"
#include "ocs2_anymal_models/camel/generated/inertia_properties.h"
#include "ocs2_anymal_models/camel/generated/jsim.h"
#include "ocs2_anymal_models/camel/generated/transforms.h"

using namespace anymal;

class AnymalCamelSwitchedModelTests : public switched_model::TestAnymalSwitchedModel {
 public:
  AnymalCamelSwitchedModelTests()
      : TestAnymalSwitchedModel(getAnymalKinematics(AnymalModel::Camel), getAnymalKinematicsAd(AnymalModel::Camel),
                                getAnymalComModel(AnymalModel::Camel), getAnymalComModelAd(AnymalModel::Camel),
                                getWholebodyDynamics(AnymalModel::Camel)) {}
};

TEST_F(AnymalCamelSwitchedModelTests, Cost) {
  this->testCosts();
}

TEST_F(AnymalCamelSwitchedModelTests, Constraints) {
  this->testConstraints();
}

TEST_F(AnymalCamelSwitchedModelTests, Kinematics) {
  this->printKinematics();
}

TEST_F(AnymalCamelSwitchedModelTests, ComDynamics) {
  this->printComModel();
}

TEST_F(AnymalCamelSwitchedModelTests, baseDynamics) {
  this->testBaseDynamics();
}

TEST_F(AnymalCamelSwitchedModelTests, fullDynamics) {
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
  iit::camel::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::camel::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::camel::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::camel::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::camel::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);
  iit::camel::dyn::tpl::ForwardDynamics<trait_t> forwardDynamics_(inertiaProperties_, motionTransforms_);

  iit::camel::dyn::tpl::ForwardDynamics<trait_t>::JointState jointAccelerationFd;
  iit::camel::dyn::tpl::ForwardDynamics<trait_t>::Acceleration baseAccelerationFd;
  iit::camel::dyn::tpl::ForwardDynamics<trait_t>::ExtForces extForces(switched_model::base_coordinate_t::Zero());
  extForces[iit::camel::LinkIdentifiers::BASE] = forcesOnBaseInBaseFrame;
  forwardDynamics_.fd(jointAccelerationFd, baseAccelerationFd, qdBase, gravity, qj, dqj, tau, extForces);

  const auto baseAcceleration =
      this->comModel_->calculateBaseLocalAccelerations(qBase, qdBase, qj, dqj, jointAccelerationFd, forcesOnBaseInBaseFrame);

  ASSERT_TRUE(baseAcceleration.isApprox(baseAccelerationFd));
}

TEST_F(AnymalCamelSwitchedModelTests, EndeffectorOrientation) {
  this->testEndeffectorOrientation();
}

TEST_F(AnymalCamelSwitchedModelTests, EndeffectorAlignedYAxisRandomHFEKFE) {
  this->testEndeffectorAlignedYAxisRandomHFEKFE();
}

TEST_F(AnymalCamelSwitchedModelTests, EndeffectorAlignedXAxisRandomHAA) {
  this->testEndeffectorAlignedXAxisRandomHAA();
}