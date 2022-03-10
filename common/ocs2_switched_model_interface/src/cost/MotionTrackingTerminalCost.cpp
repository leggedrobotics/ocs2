//
// Created by rgrandia on 10.03.22.
//

#include "ocs2_switched_model_interface/cost/MotionTrackingTerminalCost.h"

namespace switched_model {

MotionTrackingTerminalCost::MotionTrackingTerminalCost(matrix_t Q, const SwingTrajectoryPlanner& swingTrajectoryPlanner)
    : ocs2::QuadraticStateCost(std::move(Q)), swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner) {}

MotionTrackingTerminalCost* MotionTrackingTerminalCost::clone() const {
  return new MotionTrackingTerminalCost(*this);
}

MotionTrackingTerminalCost::MotionTrackingTerminalCost(const MotionTrackingTerminalCost& other)
    : ocs2::QuadraticStateCost(other), swingTrajectoryPlannerPtr_(other.swingTrajectoryPlannerPtr_) {}

vector_t MotionTrackingTerminalCost::getStateDeviation(scalar_t time, const vector_t& state,
                                                       const ocs2::TargetTrajectories& targetTrajectories) const {
  vector_t referenceState = targetTrajectories.getDesiredState(time);
  referenceState.tail(JOINT_COORDINATE_SIZE) = swingTrajectoryPlannerPtr_->getJointPositionsReference(time);
  return state - referenceState;
}

}  // namespace switched_model