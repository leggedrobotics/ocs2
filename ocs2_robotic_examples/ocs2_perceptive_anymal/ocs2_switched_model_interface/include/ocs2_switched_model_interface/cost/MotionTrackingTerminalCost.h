//
// Created by rgrandia on 10.03.22.
//

#pragma once

#include <ocs2_core/cost/StateCost.h>

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

namespace switched_model {

class MotionTrackingTerminalCost final : public ocs2::StateCost {
 public:
  explicit MotionTrackingTerminalCost(matrix_t Q);

  ~MotionTrackingTerminalCost() override = default;
  MotionTrackingTerminalCost* clone() const override;

  /** Get cost term value */
  scalar_t getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComputation) const;

  /** Get cost term quadratic approximation */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComputation) const;

 private:
  MotionTrackingTerminalCost(const MotionTrackingTerminalCost& other);

  vector_t getStateDeviation(const vector_t& state, const ocs2::PreComputation& preComputation) const;

  matrix_t Q_;
};

}  // namespace switched_model