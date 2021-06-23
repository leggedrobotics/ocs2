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

#include "pinocchio/fwd.hpp"

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/frames.hpp"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelRbdConversions::CentroidalModelRbdConversions(PinocchioInterface& pinocchioInterface,
                                                             CentroidalModelPinocchioMapping<scalar_t>& mapping)
    : pinocchioInterfacePtr_(&pinocchioInterface), mappingPtr_(&mapping) {
  mappingPtr_->setPinocchioInterface(pinocchioInterface);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeBaseKinematicsFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                             const vector_t& jointAccelerations, Vector6& basePose,
                                                                             Vector6& baseVelocity, Vector6& baseAcceleration) {
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = mappingPtr_->getCentroidalModelInfo();
  const vector_t qPinocchio = mappingPtr_->getPinocchioJointPosition(state);

  if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
    pinocchio::computeCentroidalMap(model, data, qPinocchio);
  } else if (info.centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    auto qPinocchioSrbd = info.qPinocchioNominal;
    qPinocchioSrbd.head<6>() = qPinocchio.head<6>();
    pinocchio::computeCentroidalMap(model, data, qPinocchioSrbd);
  }
  pinocchio::updateFramePlacements(model, data);

  // Base Pose in world frame
  basePose = qPinocchio.head<6>();

  // Base Velocity in world frame
  const auto& A = mappingPtr_->getCentroidalMomentumMatrix();
  const Matrix6 Ab = A.template leftCols<6>();
  const auto& Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info.actuatedDofNum);

  const vector_t vPinocchio = mappingPtr_->getPinocchioJointVelocity(state, input);
  baseVelocity.head<3>() = vPinocchio.head<3>();
  derivativeEulerAnglesZyx_ = vPinocchio.segment<3>(3);
  getAngularVelocityInWorldFrameFromEulerAnglesZyx<scalar_t>(basePose.tail<3>(), derivativeEulerAnglesZyx_, o_baseAngularVel_);
  baseVelocity.tail<3>() = o_baseAngularVel_;

  Adot_ = pinocchio::dccrba(model, data, qPinocchio, vPinocchio);
  qb_ddot_ = Ab_inv * (info.robotMass * mappingPtr_->getNormalizedCentroidalMomentumRate(input) - Adot_ * vPinocchio -
                       Aj * jointAccelerations.head(info.actuatedDofNum));

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
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = mappingPtr_->getCentroidalModelInfo();

  vector_t qPinocchio(info.generalizedCoordinatesNum);
  qPinocchio.head<3>() = rbdState.segment<3>(3);
  qPinocchio.segment<3>(3) = rbdState.head<3>();
  qPinocchio.tail(info.actuatedDofNum) = rbdState.segment(6, info.actuatedDofNum);

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.head<3>() = rbdState.segment<3>(info.generalizedCoordinatesNum + 3);
  getEulerAnglesZyxDerivativesFromGlobalAngularVelocities<scalar_t>(
      qPinocchio.segment<3>(3), rbdState.segment<3>(info.generalizedCoordinatesNum), derivativeEulerAnglesZyx_);
  vPinocchio.segment<3>(3) = derivativeEulerAnglesZyx_;
  vPinocchio.tail(info.actuatedDofNum) = rbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

  centroidalMomentum_ = pinocchio::computeCentroidalMomentum(model, data, qPinocchio, vPinocchio);

  state.head(6) = centroidalMomentum_ / info.robotMass;
  state.segment(6, info.generalizedCoordinatesNum) = qPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                       const vector_t& jointAccelerations, vector_t& rbdState) {
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = mappingPtr_->getCentroidalModelInfo();

  Vector6 basePose, baseVelocity, baseAcceleration;
  computeBaseKinematicsFromCentroidalModel(state, input, jointAccelerations, basePose, baseVelocity, baseAcceleration);

  rbdState.head<3>() = basePose.tail<3>();
  rbdState.segment<3>(3) = basePose.head<3>();
  rbdState.segment(6, info.actuatedDofNum) << state.segment(12, info.actuatedDofNum);

  rbdState.segment<3>(info.generalizedCoordinatesNum) = baseVelocity.tail<3>();
  rbdState.segment<3>(info.generalizedCoordinatesNum + 3) = baseVelocity.head<3>();
  rbdState.tail(info.actuatedDofNum) << input.tail(info.actuatedDofNum);
}

}  // namespace ocs2