//
// Created by rgrandia on 22.09.21.
//

#pragma once

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/penalties/PenaltyBase.h>

#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Implements the conversion of the friction cone constraint of all legs into a penalty cost.
 * This is a specialization of the soft-constraint wrapping in ocs2. Here we can exploit the sparsity of the particular constraint.
 */
class FrictionConeCost final : public ocs2::StateInputCost {
 public:
  FrictionConeCost(friction_cone::Config config, const SwitchedModelModeScheduleManager& modeScheduleManager,
                   std::unique_ptr<ocs2::PenaltyBase> penaltyFunction);

  FrictionConeCost* clone() const override;

  bool isActive(scalar_t time) const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FrictionConeCost(const FrictionConeCost& rhs);

  friction_cone::Config config_;
  const SwitchedModelModeScheduleManager* modeScheduleManager_;
  std::unique_ptr<ocs2::PenaltyBase> penalty_;
};

}  // namespace switched_model
