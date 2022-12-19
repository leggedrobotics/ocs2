//
// Created by rgrandia on 26.07.21.
//

#pragma once

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Implements the a penalty cost to keep the joints within upper and lower bounds. Unbounded joints are specified through a high numeric
 * value e.g. +-1e30:
 *
 * -torqueLimits(j) < torque(j) < torqueLimits(j), for all joints j
 *
 * Implementing this penalty directly as a cost allows us to take advantage of the sparsity of these constraints that target
 * individual entries of the state with a double sided penalty.
 */
class TorqueLimitsSoftConstraint final : public ocs2::StateInputCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   *
   * @param torqueLimits : joint torque limits
   * @param settings : Settings for the penalty function.
   * @param nominalState : nominal state where cost will be zero
   * @param nominalInput : nominal input where cost will be zero
   */
  TorqueLimitsSoftConstraint(const joint_coordinate_t& torqueLimits, ocs2::RelaxedBarrierPenalty::Config settings,
                             const joint_coordinate_t& nominalTorques);

  TorqueLimitsSoftConstraint* clone() const override { return new TorqueLimitsSoftConstraint(*this); }

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

  scalar_t getValue(const joint_coordinate_t& jointTorques) const;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const VectorFunctionLinearApproximation& jointTorquesDerivative) const;

 private:
  TorqueLimitsSoftConstraint(const TorqueLimitsSoftConstraint& rhs);

  std::unique_ptr<ocs2::RelaxedBarrierPenalty> jointTorquePenalty_;
  joint_coordinate_t torqueLimits_;
  scalar_t offset_;
};

}  // namespace switched_model
