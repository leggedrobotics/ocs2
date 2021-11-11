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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "ocs2_centroidal_model/CentroidalModelRbdConversions.h"

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelRbdConversions::CentroidalModelRbdConversions(PinocchioInterface& pinocchioInterface, CentroidalModelInfo info)
    : pinocchioInterfacePtr_(&pinocchioInterface), mapping_(std::move(info)) {
  mapping_.setPinocchioInterface(pinocchioInterface);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeBaseKinematicsFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                             const vector_t& jointAccelerations, Vector6& basePose,
                                                                             Vector6& baseVelocity, Vector6& baseAcceleration) {
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = mapping_.getCentroidalModelInfo();
  const auto qPinocchio = mapping_.getPinocchioJointPosition(state);

  updateCentroidalDynamics(*pinocchioInterfacePtr_, info, qPinocchio);

  // Base Pose in world frame
  basePose = qPinocchio.head<6>();
  const auto basePosition = basePose.head<3>();
  const auto baseOrientation = basePose.tail<3>();

  // Base Velocity in world frame
  const auto& A = getCentroidalMomentumMatrix(*pinocchioInterfacePtr_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto Ab_inv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info.actuatedDofNum);

  const vector_t vPinocchio = mapping_.getPinocchioJointVelocity(state, input);
  baseVelocity.head<3>() = vPinocchio.head<3>();
  derivativeEulerAnglesZyx_ = vPinocchio.segment<3>(3);
  baseAngularVelocityInWorld_ = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(baseOrientation, derivativeEulerAnglesZyx_);
  baseVelocity.tail<3>() = baseAngularVelocityInWorld_;

  Adot_ = pinocchio::dccrba(model, data, qPinocchio, vPinocchio);
  Vector6 centroidalMomentumRate = info.robotMass * getNormalizedCentroidalMomentumRate(*pinocchioInterfacePtr_, info, input);
  centroidalMomentumRate.noalias() -= Adot_ * vPinocchio;
  centroidalMomentumRate.noalias() -= Aj * jointAccelerations.head(info.actuatedDofNum);
  qbaseDdot_.noalias() = Ab_inv * centroidalMomentumRate;

  // Base Acceleration in world frame
  baseAcceleration.head<3>() = qbaseDdot_.head<3>();
  baseAngularAccelerationInWorld_ =
      getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(baseOrientation, derivativeEulerAnglesZyx_, qbaseDdot_.tail<3>());
  baseAcceleration.tail<3>() = baseAngularAccelerationInWorld_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeCentroidalStateFromRbdModel(const vector_t& rbdState, vector_t& state) {
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = mapping_.getCentroidalModelInfo();

  vector_t qPinocchio(info.generalizedCoordinatesNum);
  qPinocchio.head<3>() = rbdState.segment<3>(3);
  qPinocchio.segment<3>(3) = rbdState.head<3>();
  qPinocchio.tail(info.actuatedDofNum) = rbdState.segment(6, info.actuatedDofNum);

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.head<3>() = rbdState.segment<3>(info.generalizedCoordinatesNum + 3);
  derivativeEulerAnglesZyx_ = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPinocchio.segment<3>(3), rbdState.segment<3>(info.generalizedCoordinatesNum));
  vPinocchio.segment<3>(3) = derivativeEulerAnglesZyx_;
  vPinocchio.tail(info.actuatedDofNum) = rbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

  centroidalMomentum_ = pinocchio::computeCentroidalMomentum(model, data, qPinocchio, vPinocchio);
  centroidal_model::getNormalizedMomentum(state, info) = centroidalMomentum_ / info.robotMass;
  centroidal_model::getGeneralizedCoordinates(state, info) = qPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                       const vector_t& jointAccelerations, vector_t& rbdState) {
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = mapping_.getCentroidalModelInfo();
  Vector6 basePose, baseVelocity, baseAcceleration;
  computeBaseKinematicsFromCentroidalModel(state, input, jointAccelerations, basePose, baseVelocity, baseAcceleration);
  rbdState.head<3>() = basePose.tail<3>();
  rbdState.segment<3>(3) = basePose.head<3>();
  rbdState.segment(6, info.actuatedDofNum) = centroidal_model::getJointAngles(state, info).head(info.actuatedDofNum);
  rbdState.segment<3>(info.generalizedCoordinatesNum) = baseVelocity.tail<3>();
  rbdState.segment<3>(info.generalizedCoordinatesNum + 3) = baseVelocity.head<3>();
  rbdState.tail(info.actuatedDofNum) = centroidal_model::getJointVelocities(input, info).head(info.actuatedDofNum);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeRbdTorqueFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                        const vector_t& jointAccelerations, vector_t& rbdTorque) {
  const auto& info = mapping_.getCentroidalModelInfo();
  const vector_t pGains = vector_t::Zero(info.generalizedCoordinatesNum);
  const vector_t dGains = vector_t::Zero(info.generalizedCoordinatesNum);
  computeRbdTorqueFromCentroidalModelPD(state, input, jointAccelerations, state, input, pGains, dGains, rbdTorque);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::computeRbdTorqueFromCentroidalModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                                          const vector_t& desiredJointAccelerations,
                                                                          const vector_t& measuredState, const vector_t& measuredInput,
                                                                          const vector_t& pGains, const vector_t& dGains,
                                                                          vector_t& rbdTorque) {
  // handles
  auto& interface = *pinocchioInterfacePtr_;
  const auto& info = mapping_.getCentroidalModelInfo();
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();

  // desired
  Vector6 desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration;
  computeBaseKinematicsFromCentroidalModel(desiredState, desiredInput, desiredJointAccelerations, desiredBasePose, desiredBaseVelocity,
                                           desiredBaseAcceleration);
  vector_t qDesired(info.generalizedCoordinatesNum), vDesired(info.generalizedCoordinatesNum), aDesired(info.generalizedCoordinatesNum);
  qDesired << desiredBasePose, centroidal_model::getJointAngles(desiredState, info);
  vDesired << desiredBaseVelocity, centroidal_model::getJointVelocities(desiredInput, info);
  aDesired << desiredBaseAcceleration, desiredJointAccelerations;
  pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints, pinocchio::Force::Zero());
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parent;
    const Vector3 translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const Matrix3 rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const Vector3 contactForce = rotationWorldFrameToJointFrame * centroidal_model::getContactForces(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce);
  }
  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parent;
    const Vector3 translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const Matrix3 rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const Vector3 contactForce = rotationWorldFrameToJointFrame * centroidal_model::getContactForces(desiredInput, i, info);
    const Vector3 contactTorque = rotationWorldFrameToJointFrame * centroidal_model::getContactTorques(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce) + contactTorque;
  }

  // measured
  const vector_t qMeasured = mapping_.getPinocchioJointPosition(measuredState);
  updateCentroidalDynamics(interface, info, qMeasured);
  const vector_t vMeasured = mapping_.getPinocchioJointVelocity(measuredState, measuredInput);

  // feedforward
  rbdTorque = pinocchio::rnea(model, data, qDesired, vDesired, aDesired, fextDesired);

  // feedback
  rbdTorque.noalias() += pGains.cwiseProduct(qDesired - qMeasured);
  rbdTorque.noalias() += dGains.cwiseProduct(vDesired - vMeasured);
}

}  // namespace ocs2
