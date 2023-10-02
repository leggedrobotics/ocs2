//
// Created by rgrandia on 26.06.20.
//

#pragma once

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Implements the collision avoidance penalty function for all collision spheres given by the precomputation.
 * Uses a Gauss-Newton approximation to generate a positive semi-definite cost Hessian w.r.t. state.
 */
class CollisionAvoidanceCost final : public ocs2::StateCost {
 public:
  CollisionAvoidanceCost(ocs2::RelaxedBarrierPenalty::Config settings);

  CollisionAvoidanceCost* clone() const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  CollisionAvoidanceCost(const CollisionAvoidanceCost& rhs);

  std::unique_ptr<ocs2::RelaxedBarrierPenalty> penalty_;
};

}  // namespace switched_model
