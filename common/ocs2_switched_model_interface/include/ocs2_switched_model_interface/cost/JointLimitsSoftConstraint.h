//
// Created by rgrandia on 26.07.21.
//

#pragma once

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenalty.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Implements the a penalty cost to keep the joints within upper and lower bounds. Unbounded joints are specified through a high numeric
 * value e.g. +-1e30:
 *
 * limits.first(j) < q_joints(j) < limits.second(j), for all joints j
 *
 * Implementing this penalty directly as a cost allows us to take advantage of the sparsity of these constraints that target
 * individual entries of the state with a double sided penalty.
 */
class JointLimitsSoftConstraint final : public ocs2::StateCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @param limits : {lower bounds, upper bounds} joint limits.
   * @param settings : Settings for the penalty function.
   */
  JointLimitsSoftConstraint(std::pair<joint_coordinate_t, joint_coordinate_t> limits, ocs2::RelaxedBarrierPenalty::Config settings);

  JointLimitsSoftConstraint* clone() const override { return new JointLimitsSoftConstraint(*this); }

  scalar_t getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

  scalar_t getValue(const joint_coordinate_t& jointPositions) const;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const joint_coordinate_t& jointPositions) const;

 private:
  JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs);

  std::unique_ptr<ocs2::RelaxedBarrierPenalty> jointPenalty_;
  std::pair<joint_coordinate_t, joint_coordinate_t> limits_;
  scalar_t offset_;
};

}  // namespace switched_model