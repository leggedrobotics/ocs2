//
// Created by rgrandia on 29.04.20.
//

#include <ocs2_switched_model_interface/constraint/FootNormalConstraint.h>

namespace switched_model {

// Static member definition
constexpr size_t FootNormalConstraint::domain_dim_;

FootNormalConstraint::FootNormalConstraint(int legNumber, FootNormalConstraintMatrix settings, const ad_com_model_t& adComModel,
                                           const ad_kinematic_model_t& adKinematicsModel, bool generateModel)
    : Base_t(constraintOrder_), settings_(std::move(settings)) {
  auto FootNormalAd = [&](const ad_vector_t& x, ad_vector_t& y) { adfunc(adComModel, adKinematicsModel, legNumber, x, y); };
  std::string modelName{"FootNormalConstraint_" + std::to_string(legNumber)};
  std::string modelFolder{"/tmp/ocs2"};
  adInterface_.reset(new ocs2::CppAdInterface(FootNormalAd, domain_dim_, modelName, modelFolder));

  if (generateModel) {
    adInterface_->createModels(order_, true);
  } else {
    adInterface_->loadModelsIfAvailable(order_, true);
  }
}

FootNormalConstraint::FootNormalConstraint(const FootNormalConstraint& rhs)
    : Base_t(constraintOrder_), settings_(rhs.settings_), adInterface_(new ocs2::CppAdInterface(*rhs.adInterface_)) {}

FootNormalConstraint* FootNormalConstraint::clone() const {
  return new FootNormalConstraint(*this);
}

scalar_array_t FootNormalConstraint::getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const {
  // Assemble input
  vector_t tapedInput(domain_dim_);
  tapedInput << time, state, input;

  // Compute foot position and velocity
  const vector_t funcVal = adInterface_->getFunctionValue(tapedInput);

  // Change to std::vector
  scalar_array_t constraintValue(1);
  constraintValue[0] = settings_.constant;
  constraintValue[0] += settings_.positionMatrix.dot(funcVal.topRows(3));
  constraintValue[0] += settings_.velocityMatrix.dot(funcVal.bottomRows(3));
  return constraintValue;
}

auto FootNormalConstraint::getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const
    -> LinearApproximation_t {
  // Assemble input
  vector_t tapedInput(domain_dim_);
  tapedInput << time, state, input;

  // Compute end effector velocity and derivatives
  const auto stateInputJacobian = adInterface_->getJacobian(tapedInput);

  // Collect constraint terms
  const auto dhdx = stateInputJacobian.template middleCols<STATE_DIM>(1);
  const auto dhdu = stateInputJacobian.template rightCols<INPUT_DIM>();

  // Convert to output format
  LinearApproximation_t linearApproximation;
  linearApproximation.constraintValues = getValue(time, state, input);

  vector_t positionVelocityMatrix(6);
  positionVelocityMatrix << settings_.positionMatrix.transpose(), settings_.velocityMatrix.transpose();
  linearApproximation.derivativeState.emplace_back(positionVelocityMatrix.transpose() * dhdx);
  linearApproximation.derivativeInput.emplace_back(positionVelocityMatrix.transpose() * dhdu);

  return linearApproximation;
}

void FootNormalConstraint::adfunc(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, int legNumber,
                                  const ad_vector_t& tapedInput, ad_vector_t& o_footPositionVelocity) {
  // Extract elements from taped input
  ad_scalar_t t = tapedInput(0);
  comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
  comkino_state_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

  // Extract elements from state
  const base_coordinate_ad_t comPose = getComPose(x);
  const base_coordinate_ad_t com_comTwist = getComLocalVelocities(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);
  const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

  // Get base state from com state
  const base_coordinate_ad_t basePose = adComModel.calculateBasePose(comPose);
  const base_coordinate_ad_t com_baseTwist = adComModel.calculateBaseLocalVelocities(com_comTwist);

  const auto o_footPosition = adKinematicsModel.footPositionInOriginFrame(legNumber, basePose, qJoints);
  const auto o_footVelocity = adKinematicsModel.footVelocityInOriginFrame(legNumber, basePose, com_baseTwist, qJoints, dqJoints);

  o_footPositionVelocity.resize(6);
  o_footPositionVelocity << o_footPosition, o_footVelocity;
}

}  // namespace switched_model
