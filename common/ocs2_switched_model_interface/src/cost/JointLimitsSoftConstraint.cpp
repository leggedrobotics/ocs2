//
// Created by rgrandia on 26.07.21.
//

#include "ocs2_switched_model_interface/cost/JointLimitsSoftConstraint.h"

namespace switched_model {

JointLimitsSoftConstraint::JointLimitsSoftConstraint(std::pair<joint_coordinate_t, joint_coordinate_t> limits,
                                                     ocs2::RelaxedBarrierPenalty::Config settings)
    : limits_(limits), jointPenalty_(new ocs2::RelaxedBarrierPenalty(settings)) {}

JointLimitsSoftConstraint::JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs)
    : limits_(rhs.limits_), jointPenalty_(rhs.jointPenalty_->clone()) {}

scalar_t JointLimitsSoftConstraint::getValue(const joint_coordinate_t& jointPositions) {
  joint_coordinate_t upperBoundOffset = limits_.second - jointPositions;
  joint_coordinate_t lowerBoundOffset = jointPositions - limits_.first;

  return lowerBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(hi); }).sum() +
         upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(hi); }).sum();
}

ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(const joint_coordinate_t& jointPositions) {
  joint_coordinate_t upperBoundOffset = limits_.second - jointPositions;
  joint_coordinate_t lowerBoundOffset = jointPositions - limits_.first;

  ScalarFunctionQuadraticApproximation cost;
  cost.f = lowerBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(hi); }).sum() +
           upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getValue(hi); }).sum();

  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdx.segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE) = lowerBoundOffset.unaryExpr([&](scalar_t hi) {
    return jointPenalty_->getDerivative(hi);
  }) - upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getDerivative(hi); });

  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);
  cost.dfdxx.diagonal().segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE) = lowerBoundOffset.unaryExpr([&](scalar_t hi) {
    return jointPenalty_->getSecondDerivative(hi);
  }) + upperBoundOffset.unaryExpr([&](scalar_t hi) { return jointPenalty_->getSecondDerivative(hi); });

  return cost;
}

}  // namespace switched_model
