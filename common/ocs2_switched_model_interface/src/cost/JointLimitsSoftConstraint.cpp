//
// Created by rgrandia on 26.07.21.
//

#include "ocs2_switched_model_interface/cost/JointLimitsSoftConstraint.h"

namespace switched_model {

JointLimitsSoftConstraint::JointLimitsSoftConstraint(std::pair<joint_coordinate_t, joint_coordinate_t> limits,
                                                     ocs2::RelaxedBarrierPenalty::Config settings)
    : limits_(limits), jointPenalty_(new ocs2::RelaxedBarrierPenalty(settings)), offset_(0.0) {
  // Obtain the offset at zero joint angles. Just to compensate high negative costs when being far away from an infinite joint limit
  offset_ = -getValue(joint_coordinate_t::Zero());
}

JointLimitsSoftConstraint::JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs)
    : limits_(rhs.limits_), jointPenalty_(rhs.jointPenalty_->clone()), offset_(rhs.offset_) {}

scalar_t JointLimitsSoftConstraint::getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                                             const ocs2::PreComputation& preComp) const {
  return getValue(getJointPositions(comkino_state_t(state)));
}

ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories, const ocs2::PreComputation& preComp) const {
  return getQuadraticApproximation(getJointPositions(comkino_state_t(state)));
}

scalar_t JointLimitsSoftConstraint::getValue(const joint_coordinate_t& jointPositions) const {
  joint_coordinate_t upperBoundOffset = limits_.second - jointPositions;
  joint_coordinate_t lowerBoundOffset = jointPositions - limits_.first;

  return lowerBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(0.0, hi); }).sum() +
         upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(0.0, hi); }).sum() + offset_;
}

ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(const joint_coordinate_t& jointPositions) const {
  joint_coordinate_t upperBoundOffset = limits_.second - jointPositions;
  joint_coordinate_t lowerBoundOffset = jointPositions - limits_.first;

  ScalarFunctionQuadraticApproximation cost;
  cost.f = lowerBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(0.0, hi); }).sum() +
           upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(0.0, hi); }).sum() + offset_;

  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdx.segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE) = lowerBoundOffset.unaryExpr([&](scalar_t hi) {
    return jointPenalty_->getDerivative(0.0, hi);
  }) - upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getDerivative(0.0, hi); });

  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);
  cost.dfdxx.diagonal().segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE) = lowerBoundOffset.unaryExpr([&](scalar_t hi) {
    return jointPenalty_->getSecondDerivative(0.0, hi);
  }) + upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getSecondDerivative(0.0, hi); });

  return cost;
}

}  // namespace switched_model
