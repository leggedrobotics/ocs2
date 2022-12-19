//
// Created by rgrandia on 26.06.20.
//

#pragma once

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Implements footplacement penalty function for a provided polygon.
 * Uses a Gauss-Newton approximation to generate a positive semi-definite cost Hessian w.r.t. state.
 */
class FootPlacementCost final : public ocs2::StateCost {
 public:
  FootPlacementCost(ocs2::RelaxedBarrierPenalty::Config settings);

  FootPlacementCost* clone() const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FootPlacementCost(const FootPlacementCost& rhs);

  std::unique_ptr<ocs2::RelaxedBarrierPenalty> polygonPenalty_;
};

}  // namespace switched_model
