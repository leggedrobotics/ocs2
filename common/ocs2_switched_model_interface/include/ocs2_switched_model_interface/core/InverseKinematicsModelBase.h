//
// Created by rgrandia on 02.08.22.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

class InverseKinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using joint_jacobian_block_t = Eigen::Matrix<scalar_t, 6, 3>;

  virtual ~InverseKinematicsModelBase() = default;

  virtual vector3_t getLimbJointPositionsFromPositionBaseToFootInBaseFrame(size_t footIndex,
                                                                           const vector3_t& positionBaseToFootInBaseFrame) const = 0;

  virtual vector3_t getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(size_t footIndex,
                                                                               const vector3_t& footVelocityRelativeToBaseInBaseFrame,
                                                                               const joint_jacobian_block_t& jointJacobian,
                                                                               scalar_t damping) const = 0;

  virtual InverseKinematicsModelBase* clone() const = 0;
};

};  // namespace switched_model