//
// Created by rgrandia on 26.07.21.
//

#include "ocs2_switched_model_interface/cost/JointLimitsSoftConstraint.h"

namespace switched_model {

JointLimitsSoftConstraint::JointLimitsSoftConstraint(const std::pair<joint_coordinate_t, joint_coordinate_t>& positionlimits,
                                                     const joint_coordinate_t& velocityLimits,
                                                     ocs2::RelaxedBarrierPenalty::Config positionSettings,
                                                     ocs2::RelaxedBarrierPenalty::Config velocitySettings)
    : jointPositionPenalty_(new ocs2::RelaxedBarrierPenalty(positionSettings)),
      jointVelocityPenalty_(new ocs2::RelaxedBarrierPenalty(velocitySettings)),
      positionlimitsLimits_(positionlimits),
      velocityLimits_(velocityLimits.cwiseAbs()),
      offset_(0.0) {
  // Obtain the offset at zero joint angles. Just to compensate high negative costs when being far away from an infinite joint limit
  offset_ = -getValue(joint_coordinate_t::Zero(), joint_coordinate_t::Zero());
}

JointLimitsSoftConstraint::JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs)
    : jointPositionPenalty_(rhs.jointPositionPenalty_->clone()),
      jointVelocityPenalty_(rhs.jointVelocityPenalty_->clone()),
      positionlimitsLimits_(rhs.positionlimitsLimits_),
      velocityLimits_(rhs.velocityLimits_),
      offset_(rhs.offset_) {}

scalar_t JointLimitsSoftConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                             const ocs2::TargetTrajectories& targetTrajectories,
                                             const ocs2::PreComputation& preComp) const {
  return getValue(getJointPositions(comkino_state_t(state)), getJointVelocities(comkino_input_t(input)));
}

ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
    const ocs2::PreComputation& preComp) const {
  return getQuadraticApproximation(getJointPositions(comkino_state_t(state)), getJointVelocities(comkino_input_t(input)));
}

scalar_t JointLimitsSoftConstraint::getValue(const joint_coordinate_t& jointPositions, const joint_coordinate_t& jointVelocities) const {
  const joint_coordinate_t upperBoundPositionOffset = positionlimitsLimits_.second - jointPositions;
  const joint_coordinate_t lowerBoundPositionOffset = jointPositions - positionlimitsLimits_.first;
  const joint_coordinate_t upperBoundVelocityOffset = velocityLimits_ - jointVelocities;
  const joint_coordinate_t lowerBoundVelocityOffset = velocityLimits_ + jointVelocities;  // = qd - (-qd_limit)

  return upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenalty_->getValue(0.0, hi); }).sum() +
         lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenalty_->getValue(0.0, hi); }).sum() +
         upperBoundVelocityOffset.unaryExpr([&](scalar_t hi) { return jointVelocityPenalty_->getValue(0.0, hi); }).sum() +
         lowerBoundVelocityOffset.unaryExpr([&](scalar_t hi) { return jointVelocityPenalty_->getValue(0.0, hi); }).sum() + offset_;
}

ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(const joint_coordinate_t& jointPositions,
                                                                                          const joint_coordinate_t& jointVelocities) const {
  const joint_coordinate_t upperBoundPositionOffset = positionlimitsLimits_.second - jointPositions;
  const joint_coordinate_t lowerBoundPositionOffset = jointPositions - positionlimitsLimits_.first;
  const joint_coordinate_t upperBoundVelocityOffset = velocityLimits_ - jointVelocities;
  const joint_coordinate_t lowerBoundVelocityOffset = velocityLimits_ + jointVelocities;

  ScalarFunctionQuadraticApproximation cost;
  cost.f = upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenalty_->getValue(0.0, hi); }).sum() +
           lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenalty_->getValue(0.0, hi); }).sum() +
           upperBoundVelocityOffset.unaryExpr([&](scalar_t hi) { return jointVelocityPenalty_->getValue(0.0, hi); }).sum() +
           lowerBoundVelocityOffset.unaryExpr([&](scalar_t hi) { return jointVelocityPenalty_->getValue(0.0, hi); }).sum() + offset_;

  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdx.segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE) = lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) {
    return jointPositionPenalty_->getDerivative(0.0, hi);
  }) - upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenalty_->getDerivative(0.0, hi); });

  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);
  cost.dfdxx.diagonal().segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE) = lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) {
    return jointPositionPenalty_->getSecondDerivative(0.0, hi);
  }) + upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenalty_->getSecondDerivative(0.0, hi); });

  cost.dfdu = vector_t::Zero(INPUT_DIM);
  cost.dfdu.segment<JOINT_COORDINATE_SIZE>(3 * NUM_CONTACT_POINTS) = lowerBoundVelocityOffset.unaryExpr([&](scalar_t hi) {
    return jointVelocityPenalty_->getDerivative(0.0, hi);
  }) - upperBoundVelocityOffset.unaryExpr([&](scalar_t hi) { return jointVelocityPenalty_->getDerivative(0.0, hi); });

  cost.dfduu = matrix_t::Zero(INPUT_DIM, INPUT_DIM);
  cost.dfduu.diagonal().segment<JOINT_COORDINATE_SIZE>(3 * NUM_CONTACT_POINTS) = lowerBoundVelocityOffset.unaryExpr([&](scalar_t hi) {
    return jointVelocityPenalty_->getSecondDerivative(0.0, hi);
  }) + upperBoundVelocityOffset.unaryExpr([&](scalar_t hi) { return jointVelocityPenalty_->getSecondDerivative(0.0, hi); });

  cost.dfdux = matrix_t::Zero(INPUT_DIM, STATE_DIM);

  return cost;
}

}  // namespace switched_model
