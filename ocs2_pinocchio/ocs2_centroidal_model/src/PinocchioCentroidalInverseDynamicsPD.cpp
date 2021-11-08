/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_centroidal_model/PinocchioCentroidalInverseDynamicsPD.h"

#include <pinocchio/algorithm/rnea.hpp>

#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalInverseDynamicsPD::PinocchioCentroidalInverseDynamicsPD(PinocchioInterface& pinocchioInterface,
                                                                           const CentroidalModelInfo& centroidalModelInfo)
    : pinocchioInterfacePtr_(&pinocchioInterface),
      centroidalModelPinocchioMapping_(centroidalModelInfo),
      centroidalModelRbdConversions_(pinocchioInterface, centroidalModelInfo) {
  centroidalModelPinocchioMapping_.setPinocchioInterface(pinocchioInterface);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioCentroidalInverseDynamicsPD::getTorque(const vector_t& desiredState, const vector_t& desiredInput,
                                                         const vector_t& desiredJointAccelerations, const vector_t& measuredState,
                                                         const vector_t& measuredInput) {
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;

  // handles
  auto& interface = *pinocchioInterfacePtr_;
  const auto& info = centroidalModelPinocchioMapping_.getCentroidalModelInfo();
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();

  // desired
  CentroidalModelRbdConversions::Vector6 desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration;
  centroidalModelRbdConversions_.computeBaseKinematicsFromCentroidalModel(desiredState, desiredInput, desiredJointAccelerations,
                                                                          desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration);
  vector_t qDesired(info.generalizedCoordinatesNum), vDesired(info.generalizedCoordinatesNum), aDesired(info.generalizedCoordinatesNum);
  qDesired << desiredBasePose, centroidal_model::getJointAngles(desiredState, info);
  vDesired << desiredBaseVelocity, centroidal_model::getJointVelocities(desiredInput, info);
  aDesired << desiredBaseAcceleration, desiredJointAccelerations;
  pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints, pinocchio::Force::Zero());
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parent;
    const vector3_t translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const matrix3_t rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const vector3_t contactForce = rotationWorldFrameToJointFrame * centroidal_model::getContactForces(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce);
  }
  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parent;
    const vector3_t translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const matrix3_t rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const vector3_t contactForce = rotationWorldFrameToJointFrame * centroidal_model::getContactForces(desiredInput, i, info);
    const vector3_t contactTorque = rotationWorldFrameToJointFrame * centroidal_model::getContactTorques(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce) + contactTorque;
  }

  // measured
  const vector_t qMeasured = centroidalModelPinocchioMapping_.getPinocchioJointPosition(measuredState);
  updateCentroidalDynamics(interface, info, qMeasured);
  const vector_t vMeasured = centroidalModelPinocchioMapping_.getPinocchioJointVelocity(measuredState, measuredInput);

  // feedforward
  vector_t torque = pinocchio::rnea(model, data, qDesired, vDesired, aDesired, fextDesired);

  // feedback
  torque.noalias() += pGains_.cwiseProduct(qDesired - qMeasured);
  torque.noalias() += dGains_.cwiseProduct(vDesired - vMeasured);

  return torque;
}

}  // namespace ocs2
