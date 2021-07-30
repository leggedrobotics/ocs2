//
// Created by rgrandia on 26.07.21.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenalty.h>

namespace switched_model {

class JointLimitsSoftConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  JointLimitsSoftConstraint(std::pair<joint_coordinate_t, joint_coordinate_t> limits, ocs2::RelaxedBarrierPenalty::Config settings);

  JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs);

  scalar_t getValue(const joint_coordinate_t& jointPositions);
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const joint_coordinate_t& jointPositions);

 private:
  std::unique_ptr<ocs2::RelaxedBarrierPenalty> jointPenalty_;
  std::pair<joint_coordinate_t, joint_coordinate_t> limits_;
};

}  // namespace switched_model