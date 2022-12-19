//
// Created by rgrandia on 21.09.21.
//

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/TorqueApproximation.h>

namespace switched_model {

SwitchedModelPreComputation::SwitchedModelPreComputation(const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                         const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel, ModelSettings settings)
    : swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner), robotMass_(comModel.totalMass()) {
  // intermediate linear outputs
  std::string intermediateLibName = settings.robotName_ + "_Precomputation_intermediateLinearOutputs";
  auto intermediateDiffFunc = [&](const ad_vector_t& x, ad_vector_t& y) {
    // Extract elements from taped input
    comkino_state_ad_t state = x.segment(0, STATE_DIM);
    comkino_input_ad_t input = x.segment(STATE_DIM, INPUT_DIM);
    intermediateLinearOutputs(adComModel, adKinematicModel, state, input, y);
  };
  intermediateLinearOutputAdInterface_.reset(
      new ocs2::CppAdInterface(intermediateDiffFunc, STATE_DIM + INPUT_DIM, intermediateLibName, settings.autodiffLibraryFolder_));
  tapedStateInput_.resize(STATE_DIM + INPUT_DIM);

  const auto initCollisions = kinematicModel.collisionSpheresInBaseFrame(joint_coordinate_t::Zero());
  for (const auto& collision : initCollisions) {
    collisionRadii_.push_back(collision.radius);
  }

  // Resize collision containers
  const size_t maxNumCollisions = NUM_CONTACT_POINTS + collisionRadii_.size();
  collisionSpheresActive_.resize(maxNumCollisions);
  collisionSpheresInOriginFrame_.resize(maxNumCollisions);
  collisionSpheresDerivative_.resize(maxNumCollisions);

  // pre jump linear outputs
  std::string prejumpLibName = settings.robotName_ + "_Precomputation_prejumpLinearOutputs";
  auto prejumpDiffFunc = [&](const ad_vector_t& x, ad_vector_t& y) { prejumpLinearOutputs(adComModel, adKinematicModel, x, y); };
  prejumpLinearOutputAdInterface_.reset(
      new ocs2::CppAdInterface(prejumpDiffFunc, STATE_DIM, prejumpLibName, settings.autodiffLibraryFolder_));

  // Generate the models
  const bool verbose = true;
  const auto order = ocs2::CppAdInterface::ApproximationOrder::First;
  if (settings.recompileLibraries_) {
    intermediateLinearOutputAdInterface_->createModels(order, verbose);
    prejumpLinearOutputAdInterface_->createModels(order, verbose);
  } else {
    intermediateLinearOutputAdInterface_->loadModelsIfAvailable(order, verbose);
    prejumpLinearOutputAdInterface_->loadModelsIfAvailable(order, verbose);
  }
}

SwitchedModelPreComputation::SwitchedModelPreComputation(const SwitchedModelPreComputation& other)
    : ocs2::PreComputation(other),
      swingTrajectoryPlannerPtr_(other.swingTrajectoryPlannerPtr_),
      intermediateLinearOutputAdInterface_(new ocs2::CppAdInterface(*other.intermediateLinearOutputAdInterface_)),
      prejumpLinearOutputAdInterface_(new ocs2::CppAdInterface(*other.prejumpLinearOutputAdInterface_)),
      collisionRadii_(other.collisionRadii_),
      collisionSpheresActive_(other.collisionSpheresActive_),
      collisionSpheresInOriginFrame_(other.collisionSpheresInOriginFrame_),
      collisionSpheresDerivative_(other.collisionSpheresDerivative_),
      tapedStateInput_(other.tapedStateInput_),
      robotMass_(other.robotMass_) {}

void SwitchedModelPreComputation::request(ocs2::RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  updateFeetPhases(t);

  if (request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    updateMotionReference(t);

    tapedStateInput_ << x, u;

    updateIntermediateLinearOutputs(t, tapedStateInput_);
    if (request.contains(ocs2::Request::Approximation)) {
      updateIntermediateLinearOutputDerivatives(t, tapedStateInput_);
    }
  }
}

void SwitchedModelPreComputation::requestPreJump(ocs2::RequestSet request, scalar_t t, const vector_t& x) {
  updateFeetPhases(t);

  if (request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    updateMotionReference(t);

    updatePrejumpLinearOutputs(t, x);
    if (request.contains(ocs2::Request::Approximation)) {
      updatePrejumpLinearOutputDerivatives(t, x);
    }
  }
}

void SwitchedModelPreComputation::requestFinal(ocs2::RequestSet request, scalar_t t, const vector_t& x) {
  updateFeetPhases(t);

  if (request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    updateMotionReference(t);
  }
}

void SwitchedModelPreComputation::updateFeetPhases(scalar_t t) {
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    feetPhases_[leg] = &swingTrajectoryPlannerPtr_->getFootPhase(leg, t);
    const auto& footPhase = *feetPhases_[leg];
    contactFlags_[leg] = footPhase.contactFlag();
    surfaceNormalsInOriginFrame_[leg] = footPhase.normalDirectionInWorldFrame(t);
    footTangentialConstraintInWorldFrame_[leg] = footPhase.getFootTangentialConstraintInWorldFrame();
  }
}

void SwitchedModelPreComputation::updateMotionReference(scalar_t t) {
  // Interpolate reference
  stateReference_ = swingTrajectoryPlannerPtr_->getTargetTrajectories().getDesiredState(t);
  vector_t uRef = swingTrajectoryPlannerPtr_->getTargetTrajectories().getDesiredInput(t);

  // Extract elements from reference
  const auto basePose = getBasePose(stateReference_);
  const auto baseTwist = getBaseLocalVelocities(stateReference_);
  const auto eulerAngles = getOrientation(basePose);
  const auto qJoints = getJointPositions(stateReference_);
  const auto dqJoints = getJointVelocities(uRef);

  // If the contact force reference has zero values, overwrite it.
  if (uRef.head<3 * NUM_CONTACT_POINTS>().isZero()) {
    uRef.head<3 * NUM_CONTACT_POINTS>() = weightCompensatingInputs(robotMass_, contactFlags_, eulerAngles).head<3 * NUM_CONTACT_POINTS>();
  }

  motionReference_.eulerXYZ = eulerAngles;
  motionReference_.comPosition = getPositionInOrigin(basePose);
  motionReference_.comAngularVelocity = rotateVectorBaseToOrigin(getAngularVelocity(baseTwist), eulerAngles);
  motionReference_.comLinearVelocity = rotateVectorBaseToOrigin(getLinearVelocity(baseTwist), eulerAngles);
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto& footPhase = getFootPhase(leg);  // already updated in updateFeetPhases
    motionReference_.jointPosition[leg] = qJoints.template segment<3>(3 * leg);
    motionReference_.footPosition[leg] = footPhase.getPositionInWorld(t);
    motionReference_.jointVelocity[leg] = dqJoints.template segment<3>(3 * leg);
    motionReference_.footVelocity[leg] = footPhase.getVelocityInWorld(t);
    motionReference_.contactForce[leg] = uRef.template segment<3>(3 * leg);
  }
}

void SwitchedModelPreComputation::intermediateLinearOutputs(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                                                            const ad_vector_t& state, const ad_vector_t& input, ad_vector_t& outputs) {
  // Copy to fixed size
  comkino_state_ad_t x = state;
  comkino_input_ad_t u = input;

  // Extract elements from state
  const base_coordinate_ad_t basePose = getBasePose(x);
  const base_coordinate_ad_t baseTwist = getBaseLocalVelocities(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);
  const joint_coordinate_ad_t dqJoints = getJointVelocities(u);
  const feet_array_t<vector3_ad_t> contactForcesInBase = toArray<ad_scalar_t>(u.head<3 * NUM_CONTACT_POINTS>());

  const auto o_feetPositionsAsArray = adKinematicsModel.feetPositionsInOriginFrame(basePose, qJoints);
  const auto o_footVelocitiesAsArray = adKinematicsModel.feetVelocitiesInOriginFrame(basePose, baseTwist, qJoints, dqJoints);
  const auto jointTorques = torqueApproximation(qJoints, contactForcesInBase, adKinematicsModel);
  const auto o_collisions = adKinematicsModel.collisionSpheresInOriginFrame(basePose, qJoints);

  const int numberOfOutputs = 3 * NUM_CONTACT_POINTS + 3 * NUM_CONTACT_POINTS + JOINT_COORDINATE_SIZE + 3 * o_collisions.size();
  outputs.resize(numberOfOutputs);

  outputs.head<3 * NUM_CONTACT_POINTS>() = fromArray(o_feetPositionsAsArray);
  outputs.segment<3 * NUM_CONTACT_POINTS>(3 * NUM_CONTACT_POINTS) = fromArray(o_footVelocitiesAsArray);
  outputs.segment<JOINT_COORDINATE_SIZE>(3 * NUM_CONTACT_POINTS + 3 * NUM_CONTACT_POINTS) = jointTorques;

  int i = 3 * NUM_CONTACT_POINTS + 3 * NUM_CONTACT_POINTS + JOINT_COORDINATE_SIZE;
  for (const auto& sphere : o_collisions) {
    outputs.segment<3>(i) = sphere.position;
    i += 3;
  }
}

void SwitchedModelPreComputation::updateIntermediateLinearOutputs(scalar_t t, const vector_t& tapedStateInput) {
  // Evaluate autodiff
  const auto intermediateLinearOutputs = intermediateLinearOutputAdInterface_->getFunctionValue(tapedStateInput);

  // Read feet positions, add swing feet as collision bodies
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * leg;
    feetPositionInOriginFrame_[leg] = intermediateLinearOutputs.segment<3>(indexInOutputs);
    const auto& footPhase = getFootPhase(leg);
    collisionSpheresActive_[leg] = !footPhase.contactFlag();
    if (collisionSpheresActive_[leg]) {
      collisionSpheresInOriginFrame_[leg].position = feetPositionInOriginFrame_[leg];
      collisionSpheresInOriginFrame_[leg].radius = footPhase.getMinimumFootClearance(t);
    }
  }

  // Read feet Velocities
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * (NUM_CONTACT_POINTS + leg);
    feetVelocitiesInOriginFrame_[leg] = intermediateLinearOutputs.segment<3>(indexInOutputs);
  }

  // Read joint Torques
  jointTorques_ = intermediateLinearOutputs.segment<JOINT_COORDINATE_SIZE>(3 * NUM_CONTACT_POINTS + 3 * NUM_CONTACT_POINTS);

  // Read collision bodies (excluding the feet)
  for (int collisionIndex = 0; collisionIndex < collisionRadii_.size(); ++collisionIndex) {
    const int indexInOutputs = JOINT_COORDINATE_SIZE + 3 * (NUM_CONTACT_POINTS + NUM_CONTACT_POINTS + collisionIndex);
    const auto collisionGlobalId = NUM_CONTACT_POINTS + collisionIndex;
    collisionSpheresActive_[collisionGlobalId] = true;
    collisionSpheresInOriginFrame_[collisionGlobalId].position = intermediateLinearOutputs.segment<3>(indexInOutputs);
    collisionSpheresInOriginFrame_[collisionGlobalId].radius = collisionRadii_[collisionIndex];
  }
}

void SwitchedModelPreComputation::updateIntermediateLinearOutputDerivatives(scalar_t t, const vector_t& tapedStateInput) {
  // Evaluate autodiff
  const auto intermediateLinearOutputDerivatives = intermediateLinearOutputAdInterface_->getJacobian(tapedStateInput);

  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * leg;
    feetPositionInOriginFrameStateDerivative_[leg] = intermediateLinearOutputDerivatives.block<3, STATE_DIM>(indexInOutputs, 0);
    const auto& footPhase = getFootPhase(leg);
    collisionSpheresActive_[leg] = !footPhase.contactFlag();
    if (collisionSpheresActive_[leg]) {
      collisionSpheresDerivative_[leg] = feetPositionInOriginFrameStateDerivative_[leg];
    }
  }

  // Read feet Velocities
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * (NUM_CONTACT_POINTS + leg);
    feetVelocitiesInOriginFrameDerivative_[leg].f = feetVelocitiesInOriginFrame_[leg];
    feetVelocitiesInOriginFrameDerivative_[leg].dfdx = intermediateLinearOutputDerivatives.block<3, STATE_DIM>(indexInOutputs, 0);
    feetVelocitiesInOriginFrameDerivative_[leg].dfdu = intermediateLinearOutputDerivatives.block<3, INPUT_DIM>(indexInOutputs, STATE_DIM);
  }

  // Read joint Torques
  jointTorquesDerivative_.f = jointTorques_;
  jointTorquesDerivative_.dfdx =
      intermediateLinearOutputDerivatives.block<JOINT_COORDINATE_SIZE, STATE_DIM>(3 * NUM_CONTACT_POINTS + 3 * NUM_CONTACT_POINTS, 0);
  jointTorquesDerivative_.dfdu = intermediateLinearOutputDerivatives.block<JOINT_COORDINATE_SIZE, INPUT_DIM>(
      3 * NUM_CONTACT_POINTS + 3 * NUM_CONTACT_POINTS, STATE_DIM);

  // Read collision bodies (excluding the feet)
  for (int collisionIndex = 0; collisionIndex < collisionRadii_.size(); ++collisionIndex) {
    const int indexInOutputs = JOINT_COORDINATE_SIZE + 3 * (NUM_CONTACT_POINTS + NUM_CONTACT_POINTS + collisionIndex);
    const auto collisionGlobalId = NUM_CONTACT_POINTS + collisionIndex;
    collisionSpheresActive_[collisionGlobalId] = true;
    collisionSpheresDerivative_[collisionGlobalId] = intermediateLinearOutputDerivatives.block<3, STATE_DIM>(indexInOutputs, 0);
  }
}

void SwitchedModelPreComputation::prejumpLinearOutputs(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                                                       const ad_vector_t& state, ad_vector_t& outputs) {
  // Copy to fixed size
  comkino_state_ad_t x = state;

  // Extract elements from state
  const base_coordinate_ad_t basePose = getBasePose(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);

  const auto o_feetPositionsAsArray = adKinematicsModel.feetPositionsInOriginFrame(basePose, qJoints);

  const int numberOfOutputs = 3 * NUM_CONTACT_POINTS;
  outputs.resize(numberOfOutputs);

  outputs.head<3 * NUM_CONTACT_POINTS>() = fromArray(o_feetPositionsAsArray);
}

void SwitchedModelPreComputation::updatePrejumpLinearOutputs(scalar_t t, const vector_t& state) {
  // Evaluate autodiff
  const auto prejumpLinearOutputs = prejumpLinearOutputAdInterface_->getFunctionValue(state);

  // Read feet positions, add swing feet as collision bodies
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * leg;
    feetPositionInOriginFrame_[leg] = prejumpLinearOutputs.segment<3>(indexInOutputs);
  }
}

void SwitchedModelPreComputation::updatePrejumpLinearOutputDerivatives(scalar_t t, const vector_t& state) {
  // Evaluate autodiff
  const auto prejumpLinearOutputDerivatives = prejumpLinearOutputAdInterface_->getJacobian(state);

  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * leg;
    feetPositionInOriginFrameStateDerivative_[leg] = prejumpLinearOutputDerivatives.block<3, STATE_DIM>(indexInOutputs, 0);
  }
}

}  // namespace switched_model