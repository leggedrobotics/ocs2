//
// Created by rgrandia on 10.03.22.
//

#pragma once

#include <ocs2_core/cost/QuadraticStateCost.h>

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

namespace switched_model {

class MotionTrackingTerminalCost : public ocs2::QuadraticStateCost {
 public:
  MotionTrackingTerminalCost(matrix_t Q, const SwingTrajectoryPlanner& swingTrajectoryPlanner);

  ~MotionTrackingTerminalCost() override = default;
  MotionTrackingTerminalCost* clone() const override;

 protected:
  MotionTrackingTerminalCost(const MotionTrackingTerminalCost& other);

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories) const override;

 private:
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
};

}  // namespace switched_model