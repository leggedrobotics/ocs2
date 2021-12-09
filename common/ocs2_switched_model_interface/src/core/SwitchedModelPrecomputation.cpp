//
// Created by rgrandia on 21.09.21.
//

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"

#include <ocs2_switched_model_interface/core/TorqueApproximation.h>

namespace switched_model {

SwitchedModelPreComputation::SwitchedModelPreComputation(const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                         const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel, ModelSettings settings)
    : swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner) {
  std::string libName = "AnymalPrecomputation_intermediateLinearOutputs";
  std::string libFolder = "/tmp/ocs2";
  auto diffFunc = [&](const ad_vector_t& x, ad_vector_t& y) {
    // Extract elements from taped input
    comkino_state_ad_t state = x.segment(0, STATE_DIM);
    comkino_input_ad_t input = x.segment(STATE_DIM, INPUT_DIM);
    intermediateLinearOutputs(adComModel, adKinematicModel, state, input, y);
  };
  intermediateLinearOutputAdInterface_.reset(new ocs2::CppAdInterface(diffFunc, STATE_DIM + INPUT_DIM, libName, libFolder));

  const auto initCollisions = kinematicModel.collisionSpheresInBaseFrame(joint_coordinate_t::Zero());
  for (const auto& collision : initCollisions) {
    collisionRadii_.push_back(collision.radius);
  }

  // Generate the model
  const bool verbose = true;
  const auto order = ocs2::CppAdInterface::ApproximationOrder::First;
  if (settings.recompileLibraries_) {
    intermediateLinearOutputAdInterface_->createModels(order, verbose);
  } else {
    intermediateLinearOutputAdInterface_->loadModelsIfAvailable(order, verbose);
  }
}

SwitchedModelPreComputation::SwitchedModelPreComputation(const SwitchedModelPreComputation& other)
    : ocs2::PreComputation(other),
      swingTrajectoryPlannerPtr_(other.swingTrajectoryPlannerPtr_),
      intermediateLinearOutputAdInterface_(new ocs2::CppAdInterface(*other.intermediateLinearOutputAdInterface_)),
      collisionRadii_(other.collisionRadii_) {}

void SwitchedModelPreComputation::request(ocs2::RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  updateFeetPhases(t);

  if (request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    updateIntermediateLinearOutputs(t, x, u);
    if (request.contains(ocs2::Request::Approximation)) {
      updateIntermediateLinearOutputDerivatives(t, x, u);
    }
  }
}

void SwitchedModelPreComputation::requestPreJump(ocs2::RequestSet request, scalar_t t, const vector_t& x) {
  updateFeetPhases(t);
}

void SwitchedModelPreComputation::requestFinal(ocs2::RequestSet request, scalar_t t, const vector_t& x) {
  updateFeetPhases(t);
}

void SwitchedModelPreComputation::updateFeetPhases(scalar_t t) {
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    feetPhases_[leg] = &swingTrajectoryPlannerPtr_->getFootPhase(leg, t);
    contactFlags_[leg] = feetPhases_[leg]->contactFlag();
    surfaceNormalsInOriginFrame_[leg] = feetPhases_[leg]->normalDirectionInWorldFrame(t);
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

void SwitchedModelPreComputation::updateIntermediateLinearOutputs(scalar_t t, const vector_t& x, const vector_t& u) {
  // Clear old precomputation
  collisionSpheresInOriginFrame_.clear();

  // Evaluate autodiff
  const vector_t tapedInput = (vector_t(x.rows() + u.rows()) << x, u).finished();
  const auto intermediateLinearOutputs = intermediateLinearOutputAdInterface_->getFunctionValue(tapedInput);

  // Read feet positions, add swing feet as collision bodies
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * leg;
    feetPositionInOriginFrame_[leg] = intermediateLinearOutputs.segment<3>(indexInOutputs);
    const auto& footPhase = getFootPhase(leg);
    if (!footPhase.contactFlag()) {
      collisionSpheresInOriginFrame_.push_back({feetPositionInOriginFrame_[leg], footPhase.getMinimumFootClearance(t)});
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
    collisionSpheresInOriginFrame_.push_back({intermediateLinearOutputs.segment<3>(indexInOutputs), collisionRadii_[collisionIndex]});
  }
}

void SwitchedModelPreComputation::updateIntermediateLinearOutputDerivatives(scalar_t t, const vector_t& x, const vector_t& u) {
  // Clear old precomputation
  collisionSpheresDerivative_.clear();

  // Evaluate autodiff
  const vector_t tapedInput = (vector_t(x.rows() + u.rows()) << x, u).finished();
  const auto intermediateLinearOutputDerivatives = intermediateLinearOutputAdInterface_->getJacobian(tapedInput);

  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int indexInOutputs = 3 * leg;
    feetPositionInOriginFrameStateDerivative_[leg] = intermediateLinearOutputDerivatives.block<3, STATE_DIM>(indexInOutputs, 0);
    const auto& footPhase = getFootPhase(leg);
    if (!footPhase.contactFlag()) {
      collisionSpheresDerivative_.push_back(feetPositionInOriginFrameStateDerivative_[leg]);
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
    collisionSpheresDerivative_.push_back(intermediateLinearOutputDerivatives.block<3, STATE_DIM>(indexInOutputs, 0));
  }
}

}  // namespace switched_model