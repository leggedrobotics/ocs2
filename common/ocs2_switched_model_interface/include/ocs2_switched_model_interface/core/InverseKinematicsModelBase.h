//
// Created by rgrandia on 02.08.22.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

class InverseKinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~InverseKinematicsModelBase() = default;

  virtual vector3_t getLimbJointPositionsFromPositionBaseToFootInBaseFrame(size_t footIndex,
                                                                           const vector3_t& positionBaseToFootInBaseFrame) const = 0;

  virtual InverseKinematicsModelBase* clone() const = 0;
};

};  // namespace switched_model