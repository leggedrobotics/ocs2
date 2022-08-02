//
// Created by rgrandia on 02.08.22.
//

#pragma once

#include <ocs2_switched_model_interface/core/InverseKinematicsModelBase.h>

#include <ocs2_switched_model_interface/analytical_inverse_kinematics/LegInverseKinematicParameters.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_anymal_models/QuadrupedPinocchioMapping.h"

namespace anymal {

class QuadrupedInverseKinematics final : public switched_model::InverseKinematicsModelBase {
 public:
  QuadrupedInverseKinematics(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);

  ~QuadrupedInverseKinematics() override = default;

  QuadrupedInverseKinematics* clone() const override;

  switched_model::vector3_t getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      size_t footIndex, const switched_model::vector3_t& positionBaseToFootInBaseFrame) const override;

 private:
  QuadrupedInverseKinematics(const QuadrupedInverseKinematics& other) = default;

  switched_model::feet_array_t<switched_model::analytical_inverse_kinematics::LegInverseKinematicParameters> parameters_;
};

};  // namespace anymal