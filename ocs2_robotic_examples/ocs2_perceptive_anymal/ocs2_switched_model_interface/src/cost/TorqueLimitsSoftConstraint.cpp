//
// Created by rgrandia on 26.07.21.
//

#include "ocs2_switched_model_interface/cost/TorqueLimitsSoftConstraint.h"

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"
#include "ocs2_switched_model_interface/core/TorqueApproximation.h"

namespace switched_model {

TorqueLimitsSoftConstraint::TorqueLimitsSoftConstraint(const joint_coordinate_t& torqueLimits, ocs2::RelaxedBarrierPenalty::Config settings,
                                                       const joint_coordinate_t& nominalTorques)
    : jointTorquePenalty_(new ocs2::RelaxedBarrierPenalty(settings)), torqueLimits_(torqueLimits), offset_(0.0) {
  // Obtain the offset at zero joint angles. Just to compensate high negative costs when being far away from an infinite joint limit
  offset_ = -getValue(nominalTorques);
}

TorqueLimitsSoftConstraint::TorqueLimitsSoftConstraint(const TorqueLimitsSoftConstraint& rhs)
    : jointTorquePenalty_(rhs.jointTorquePenalty_->clone()), torqueLimits_(rhs.torqueLimits_), offset_(rhs.offset_) {}

scalar_t TorqueLimitsSoftConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                              const ocs2::TargetTrajectories& targetTrajectories,
                                              const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  return getValue(switchedModelPreComp.jointTorques());
}

ScalarFunctionQuadraticApproximation TorqueLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
    const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  return getQuadraticApproximation(switchedModelPreComp.jointTorquesDerivative());
}

scalar_t TorqueLimitsSoftConstraint::getValue(const joint_coordinate_t& jointTorques) const {
  const joint_coordinate_t upperBoundTorqueOffset = torqueLimits_ - jointTorques;
  const joint_coordinate_t lowerBoundTorqueOffset = torqueLimits_ + jointTorques;  // = qd - (-qd_limit)

  return upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) { return jointTorquePenalty_->getValue(0.0, hi); }).sum() +
         lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) { return jointTorquePenalty_->getValue(0.0, hi); }).sum() + offset_;
}

ScalarFunctionQuadraticApproximation TorqueLimitsSoftConstraint::getQuadraticApproximation(
    const VectorFunctionLinearApproximation& jointTorquesDerivative) const {
  const joint_coordinate_t upperBoundTorqueOffset = torqueLimits_ - jointTorquesDerivative.f;
  const joint_coordinate_t lowerBoundTorqueOffset = torqueLimits_ + jointTorquesDerivative.f;  // = qd - (-qd_limit)

  ScalarFunctionQuadraticApproximation cost;
  cost.f = upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) { return jointTorquePenalty_->getValue(0.0, hi); }).sum() +
           lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) { return jointTorquePenalty_->getValue(0.0, hi); }).sum() + offset_;

  // Penalty derivatives w.r.t. the constraint
  const joint_coordinate_t penaltyDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) {
    return jointTorquePenalty_->getDerivative(0.0, hi);
  }) - upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) { return jointTorquePenalty_->getDerivative(0.0, hi); });
  const joint_coordinate_t penaltySecondDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) {
    return jointTorquePenalty_->getSecondDerivative(0.0, hi);
  }) + upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) { return jointTorquePenalty_->getSecondDerivative(0.0, hi); });

  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdu = vector_t::Zero(INPUT_DIM);
  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);
  cost.dfduu = matrix_t::Zero(INPUT_DIM, INPUT_DIM);
  cost.dfdux = matrix_t::Zero(INPUT_DIM, STATE_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const int torqueStartIndex = 3 * leg;
    const int forceStartIndex = 3 * leg;
    const int jointStartStateIndex = 2 * BASE_COORDINATE_SIZE + 3 * leg;

    // Shorthand for blocks of the full derivatives.
    const auto dhdjoints = jointTorquesDerivative.dfdx.block<3, 3>(torqueStartIndex, jointStartStateIndex);
    const auto dhdforce = jointTorquesDerivative.dfdu.block<3, 3>(torqueStartIndex, forceStartIndex);
    const auto dpenaltydh = penaltyDerivatives.segment<3>(torqueStartIndex);
    const auto d2penaltydh2 = penaltySecondDerivatives.segment<3>(torqueStartIndex);

    // State derivative only has elements for joints of this leg
    cost.dfdx.segment<3>(jointStartStateIndex).noalias() = dhdjoints.transpose() * dpenaltydh;
    cost.dfdxx.block<3, 3>(jointStartStateIndex, jointStartStateIndex).noalias() =
        dhdjoints.transpose() * d2penaltydh2.asDiagonal() * dhdjoints;

    // Input derivative only has elements for contact forces of this leg
    cost.dfdu.segment<3>(forceStartIndex).noalias() = dhdforce.transpose() * dpenaltydh;
    cost.dfduu.block<3, 3>(forceStartIndex, forceStartIndex).noalias() = dhdforce.transpose() * d2penaltydh2.asDiagonal() * dhdforce;

    // Cross terms
    cost.dfdux.block<3, 3>(forceStartIndex, jointStartStateIndex).noalias() = dhdforce.transpose() * d2penaltydh2.asDiagonal() * dhdjoints;
  }

  return cost;
}

}  // namespace switched_model
