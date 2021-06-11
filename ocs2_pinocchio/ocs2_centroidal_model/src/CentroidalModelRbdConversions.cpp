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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelRbdConversions::CentroidalModelRbdConversions(
    const CentroidalModelPinocchioInterface<scalar_t>& centroidalModelPinocchioInterface)
    : centroidalModelPinocchioInterface_(centroidalModelPinocchioInterface) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeBaseKinematicsFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                             const vector_t& jointAccelerations, Vector6& basePose,
                                                                             Vector6& baseVelocity, Vector6& baseAcceleration) {
  const size_t generalizedVelocityNum = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t actuatedDofNum = generalizedVelocityNum - 6;
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
              Adot_ * centroidalModelInfo.vPinocchio - centroidalModelInfo.Aj * jointAccelerations.head(actuatedDofNum));

  // Base Acceleration in world frame
  baseAcceleration.head<3>() = qb_ddot_.head<3>();
  getAngularAccelerationInWorldFrameFromEulerAnglesZyx<scalar_t>(basePose.tail<3>(), derivativeEulerAnglesZyx_, qb_ddot_.tail<3>(),
                                                                 o_baseAngularAccel_);
  baseAcceleration.tail<3>() = o_baseAngularAccel_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeCentroidalStateFromRbdModel(const vector_t& rbdState, vector_t& state) {
  const size_t generalizedVelocityNum = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t actuatedDofNum = generalizedVelocityNum - 6;
  const auto& centroidalModelInfo = centroidalModelPinocchioInterface_.getCentroidalModelInfo();

  vector_t qPinocchio(generalizedVelocityNum);
  qPinocchio.head<3>() = rbdState.segment<3>(3);
  qPinocchio.segment<3>(3) = rbdState.head<3>();
  qPinocchio.tail(actuatedDofNum) = rbdState.segment(6, actuatedDofNum);

  vector_t vPinocchio(generalizedVelocityNum);
  vPinocchio.head<3>() = rbdState.segment<3>(generalizedVelocityNum + 3);
  getEulerAnglesZyxDerivativesFromGlobalAngularVelocities<scalar_t>(qPinocchio.segment<3>(3), rbdState.segment<3>(generalizedVelocityNum),
                                                                    derivativeEulerAnglesZyx_);
  vPinocchio.segment<3>(3) = derivativeEulerAnglesZyx_;
  vPinocchio.tail(actuatedDofNum) = rbdState.segment(generalizedVelocityNum + 6, actuatedDofNum);

  centroidalMomentum_ = pinocchio::computeCentroidalMomentum(centroidalModelPinocchioInterface_.getRobotModel(),
                                                             centroidalModelPinocchioInterface_.getRobotData(), qPinocchio, vPinocchio);

  state.head(6) = centroidalMomentum_ / centroidalModelInfo.mass;
  state.segment(6, generalizedVelocityNum) = qPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                       const vector_t& jointAccelerations, vector_t& rbdState) {
  const size_t generalizedVelocityNum = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t actuatedDofNum = generalizedVelocityNum - 6;

  Vector6 basePose, baseVelocity, baseAcceleration;
  computeBaseKinematicsFromCentroidalModel(state, input, jointAccelerations, basePose, baseVelocity, baseAcceleration);

  rbdState.head<3>() = basePose.tail<3>();
  rbdState.segment<3>(3) = basePose.head<3>();
  rbdState.segment(6, actuatedDofNum) << state.segment(12, actuatedDofNum);

  rbdState.segment<3>(generalizedVelocityNum) = baseVelocity.tail<3>();
  rbdState.segment<3>(generalizedVelocityNum + 3) = baseVelocity.head<3>();
  rbdState.tail(actuatedDofNum) << input.tail(actuatedDofNum);
}

}  // namespace ocs2