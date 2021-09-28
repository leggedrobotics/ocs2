//
// Created by rgrandia on 22.09.21.
//

#pragma once

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/soft_constraint/penalties/PenaltyBase.h>

#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Implements the conversion of the friction cone constraint to a penalty cost.
 * This is a specialization of the soft-constraint wrapping in ocs2. Here we can exploit the sparsity of the particular constraint.
 */
class FrictionConeCost final : public ocs2::StateInputCost {
 public:
  FrictionConeCost(FrictionConeConstraint::Config config, int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager,
                   std::unique_ptr<ocs2::PenaltyBase> penaltyFunction);

  FrictionConeCost* clone() const override;

  bool isActive(scalar_t time) const override { return constraint_->isActive(time); }

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FrictionConeCost(const FrictionConeCost& rhs);

  int legNumber_;
  std::unique_ptr<FrictionConeConstraint> constraint_;
  std::unique_ptr<ocs2::PenaltyBase> penalty_;
};

}  // namespace switched_model