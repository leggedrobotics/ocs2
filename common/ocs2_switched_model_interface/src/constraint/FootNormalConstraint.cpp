//
// Created by rgrandia on 29.04.20.
//

#include <ocs2_switched_model_interface/constraint/FootNormalConstraint.h>

namespace switched_model {

FootNormalConstraint::FootNormalConstraint(int legNumber, settings_t settings, const ad_com_model_t& adComModel,
                                           const ad_kinematic_model_t& adKinematicsModel, bool generateModel)
    : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
      settings_(std::move(settings)),
      adComModelPtr_(adComModel.clone()),
      adKinematicsModelPtr_(adKinematicsModel.clone()),
      legNumber_(legNumber) {
  const size_t parameter_dim = 7;
  const std::string modelName{"FootNormalConstraint_" + std::to_string(legNumber)};
  const std::string modelFolder{"/tmp/ocs2"};

  initialize(STATE_DIM, INPUT_DIM, parameter_dim, modelName, modelFolder, generateModel, true);
}

FootNormalConstraint::FootNormalConstraint(const FootNormalConstraint& rhs)
    : ocs2::StateInputConstraintCppAd(rhs),
      settings_(rhs.settings_),
      adComModelPtr_(rhs.adComModelPtr_->clone()),
      adKinematicsModelPtr_(rhs.adKinematicsModelPtr_->clone()),
      legNumber_(rhs.legNumber_) {}

FootNormalConstraint* FootNormalConstraint::clone() const {
  return new FootNormalConstraint(*this);
}

vector_t FootNormalConstraint::getParameters(scalar_t time) const {
  vector_t parameters(7);
  parameters << settings_.positionMatrix.transpose(), settings_.velocityMatrix.transpose(), settings_.constant;
  return parameters;
}

ocs2::ad_vector_t FootNormalConstraint::constraintFunction(ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                                           const ocs2::ad_vector_t& parameters) const {
  // Extract elements from taped input
  const comkino_state_ad_t x = state;
  const comkino_input_ad_t u = input;

  // Unpack constraint parameters
  const auto positionMatrix = parameters.head(3);
  const auto velocityMatrix = parameters.segment(3, 3);
  const ad_scalar_t constant = parameters(6);

  // Extract elements from state
  const base_coordinate_ad_t comPose = getComPose(x);
  const base_coordinate_ad_t com_comTwist = getComLocalVelocities(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);
  const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

  // Get base state from com state
  const base_coordinate_ad_t basePose = adComModelPtr_->calculateBasePose(comPose);
  const base_coordinate_ad_t com_baseTwist = adComModelPtr_->calculateBaseLocalVelocities(com_comTwist);

  const auto o_footPosition = adKinematicsModelPtr_->footPositionInOriginFrame(legNumber_, basePose, qJoints);
  const auto o_footVelocity = adKinematicsModelPtr_->footVelocityInOriginFrame(legNumber_, basePose, com_baseTwist, qJoints, dqJoints);

  ocs2::ad_vector_t constraint(1);
  constraint << positionMatrix.dot(o_footPosition) + velocityMatrix.dot(o_footVelocity) + constant;

  return constraint;
}

}  // namespace switched_model
