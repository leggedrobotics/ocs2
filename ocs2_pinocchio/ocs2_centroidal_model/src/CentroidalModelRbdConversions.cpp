/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/algorithm/centroidal.hpp"

namespace ocs2 {

CentroidalModelRbdConversions::CentroidalModelRbdConversions(
    const CentroidalModelPinocchioInterface<scalar_t>& centroidalModelPinocchioInterface)
    : centroidalModelPinocchioInterface_(centroidalModelPinocchioInterface) {}

void CentroidalModelRbdConversions::computeBaseKinematicsFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                             const vector_t& jointAccelerations, Vector6& basePose,
                                                                             Vector6& baseVelocity, Vector6& baseAcceleration) {
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;
  const auto& centroidalModelInfo = centroidalModelPinocchioInterface_.getCentroidalModelInfo();

  centroidalModelPinocchioInterface_.updatePinocchioJointPositions(state);
  centroidalModelPinocchioInterface_.updatePinocchioJointVelocities(state, input);
  if (centroidalModelInfo.centroidalModelType ==
      CentroidalModelPinocchioInterface<scalar_t>::CentroidalModelType::SingleRigidBodyDynamics) {
    pinocchio::forwardKinematics(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData(),
                                 centroidalModelInfo.qPinocchio);
  }
  pinocchio::updateFramePlacements(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData());

  // Base Pose in world frame
  basePose = centroidalModelInfo.qPinocchio.head<6>();

  // Base Velocity in world frame
  baseVelocity.head<3>() = centroidalModelInfo.vPinocchio.head<3>();
  derivativeEulerAnglesZyx_ = centroidalModelInfo.vPinocchio.segment<3>(3);
  getAngularVelocityInWorldFrameFromEulerAnglesZyx<scalar_t>(basePose.tail<3>(), derivativeEulerAnglesZyx_, o_baseAngularVel_);
  baseVelocity.tail<3>() = o_baseAngularVel_;

  Adot_ = pinocchio::dccrba(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData(),
                            centroidalModelInfo.qPinocchio, centroidalModelInfo.vPinocchio);
  qb_ddot_ = centroidalModelInfo.Ab_inv *
             (centroidalModelInfo.mass * centroidalModelPinocchioInterface_.normalizedCentroidalMomentumRate(input) -
              Adot_ * centroidalModelInfo.vPinocchio - centroidalModelInfo.Aj * jointAccelerations.head(ACTUATED_DOF_NUM));

  // Base Acceleration in world frame
  baseAcceleration.head<3>() = qb_ddot_.head<3>();
  getAngularAccelerationInWorldFrameFromEulerAnglesZyx<scalar_t>(basePose.tail<3>(), derivativeEulerAnglesZyx_, qb_ddot_.tail<3>(),
                                                                 o_baseAngularAccel_);
  baseAcceleration.tail<3>() = o_baseAngularAccel_;
}

void CentroidalModelRbdConversions::computeCentroidalStateFromRbdModel(const vector_t& rbdState, vector_t& state) {
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;
  const auto& centroidalModelInfo = centroidalModelPinocchioInterface_.getCentroidalModelInfo();

  vector_t qPinocchio(GENERALIZED_VEL_NUM);
  qPinocchio.head<3>() = rbdState.segment<3>(3);
  qPinocchio.segment<3>(3) = rbdState.head<3>();
  qPinocchio.tail(ACTUATED_DOF_NUM) = rbdState.segment(6, ACTUATED_DOF_NUM);

  vector_t vPinocchio(GENERALIZED_VEL_NUM);
  vPinocchio.head<3>() = rbdState.segment<3>(GENERALIZED_VEL_NUM + 3);
  getEulerAnglesZyxDerivativesFromGlobalAngularVelocities<scalar_t>(qPinocchio.segment<3>(3), rbdState.segment<3>(GENERALIZED_VEL_NUM),
                                                                    derivativeEulerAnglesZyx_);
  vPinocchio.segment<3>(3) = derivativeEulerAnglesZyx_;
  vPinocchio.tail(ACTUATED_DOF_NUM) = rbdState.segment(GENERALIZED_VEL_NUM + 6, ACTUATED_DOF_NUM);

  centroidalMomentum_ = pinocchio::computeCentroidalMomentum(centroidalModelPinocchioInterface_.getRobotModel(),
                                                             centroidalModelPinocchioInterface_.getRobotData(), qPinocchio, vPinocchio);

  state.head(6) = centroidalMomentum_ / centroidalModelInfo.mass;
  state.segment(6, GENERALIZED_VEL_NUM) = qPinocchio;
}

void CentroidalModelRbdConversions::computeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                       const vector_t& jointAccelerations, vector_t& rbdState) {
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;

  Vector6 basePose, baseVelocity, baseAcceleration;
  computeBaseKinematicsFromCentroidalModel(state, input, jointAccelerations, basePose, baseVelocity, baseAcceleration);

  rbdState.head<3>() = basePose.tail<3>();
  rbdState.segment<3>(3) = basePose.head<3>();
  rbdState.segment(6, ACTUATED_DOF_NUM) << state.segment(12, ACTUATED_DOF_NUM);

  rbdState.segment<3>(GENERALIZED_VEL_NUM) = baseVelocity.tail<3>();
  rbdState.segment<3>(GENERALIZED_VEL_NUM + 3) = baseVelocity.head<3>();
  rbdState.tail(ACTUATED_DOF_NUM) << input.tail(ACTUATED_DOF_NUM);
}

}  // namespace ocs2