//
// Created by rgrandia on 26.07.21.
//

#pragma once

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenalty.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Implements the a penalty cost to keep the joints within upper and lower bounds. Unbounded joints are specified through a high numeric
 * value e.g. +-1e30:
 *
 * positionLimits.first(j) < q_joints(j) < positionLimits.second(j), for all joints j
 * -velocityLimits < dq_joints(j) < velocityLimits, for all joints j
 *
 * Implementing this penalty directly as a cost allows us to take advantage of the sparsity of these constraints that target
 * individual entries of the state with a double sided penalty.
 */
class JointLimitsSoftConstraint final : public ocs2::StateInputCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   *
   * @param positionlimits : {lower bounds, upper bounds} joint position limits.
   * @param velocityLimits : joint velocity limits
   * @param positionSettings : Settings for the position penalty function.
   * @param velocitySettings : Settings for the velocity penalty function.
   */
  JointLimitsSoftConstraint(const std::pair<joint_coordinate_t, joint_coordinate_t>& positionlimits,
                            const joint_coordinate_t& velocityLimits, ocs2::RelaxedBarrierPenalty::Config positionSettings,
                            ocs2::RelaxedBarrierPenalty::Config velocitySettings);

  JointLimitsSoftConstraint* clone() const override { return new JointLimitsSoftConstraint(*this); }

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

  scalar_t getValue(const joint_coordinate_t& jointPositions, const joint_coordinate_t& jointVelocities) const;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const joint_coordinate_t& jointPositions,
                                                                 const joint_coordinate_t& jointVelocities) const;

 private:
  JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs);

  std::unique_ptr<ocs2::RelaxedBarrierPenalty> jointPositionPenalty_;
  std::unique_ptr<ocs2::RelaxedBarrierPenalty> jointVelocityPenalty_;
  std::pair<joint_coordinate_t, joint_coordinate_t> positionlimitsLimits_;
  joint_coordinate_t velocityLimits_;
  scalar_t offset_;
};

}  // namespace switched_model