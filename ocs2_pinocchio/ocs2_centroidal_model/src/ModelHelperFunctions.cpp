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

#include "ocs2_centroidal_model/ModelHelperFunctions.h"

#include "ocs2_centroidal_model/AccessHelperFunctions.h"

#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void updateCentroidalDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q) {
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR_T, 3, 3>;
  using matrix6_t = Eigen::Matrix<SCALAR_T, 6, 6>;

  const auto& model = interface.getModel();
  auto& data = interface.getData();

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalMap(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      const vector3_t eulerAnglesZyx = q.template segment<3>(3);
      const matrix3_t mappingZyx = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAnglesZyx);
      const matrix3_t rotationBaseToWorld = getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
      const vector3_t comToBasePositionInWorld = rotationBaseToWorld * info.comToBasePositionNominal;
      const matrix3_t skewSymmetricMap = skewSymmetricMatrix(comToBasePositionInWorld);
      const matrix3_t mat1 = rotationBaseToWorld * info.centroidalInertiaNominal;
      const matrix3_t mat2 = rotationBaseToWorld.transpose() * mappingZyx;
      matrix6_t Ab = matrix6_t::Zero();
      Ab.template topLeftCorner<3, 3>().diagonal().array() = info.robotMass;
      Ab.template topRightCorner<3, 3>().noalias() = info.robotMass * skewSymmetricMap * mappingZyx;
      Ab.template bottomRightCorner<3, 3>().noalias() = mat1 * mat2;
      data.Ag = Eigen::Matrix<SCALAR_T, -1, -1>::Zero(6, info.generalizedCoordinatesNum);
      data.Ag.template leftCols<6>() = Ab;
      data.com[0] = q.template head<3>() - comToBasePositionInWorld;
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
      break;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void updateCentroidalDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v) {
  using matrix6x_t = Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  vector_t a = vector_t::Zero(info.generalizedCoordinatesNum);
  matrix6x_t dhdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdv(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotda(6, info.generalizedCoordinatesNum);

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalDynamicsDerivatives(model, data, q, v, a, dhdq, dhdotdq, dhdotdv, dhdotda);
      data.Ag = dhdotda;
      // Filling in data.dFdq is a hack since data.dhdq is not available
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = getCentroidalMomentumZyxGradient(interface, info, q, v);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      // Filling in data.dFdq is a hack since data.dhdq is not available
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = getCentroidalMomentumZyxGradient(interface, info, q, v);
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>& getCentroidalMomentumMatrix(const PinocchioInterfaceTpl<SCALAR_T>& interface) {
  return interface.getData().Ag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getPositionComToContactPointInWorldFrame(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                       const CentroidalModelInfoTpl<SCALAR_T>& info, size_t contactIndex) {
  const auto& data = interface.getData();
  return (data.oMf[info.endEffectorFrameIndices[contactIndex]].translation() - data.com[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame(
    const PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info, size_t contactIndex) {
  const auto& model = interface.getModel();
  auto data = interface.getData();
  Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> jacobianWorldToContactPointInWorldFrame;
  jacobianWorldToContactPointInWorldFrame.setZero(6, info.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, info.endEffectorFrameIndices[contactIndex], pinocchio::LOCAL_WORLD_ALIGNED,
                              jacobianWorldToContactPointInWorldFrame);
  Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> J_com = getCentroidalMomentumMatrix(interface).template topRows<3>() / info.robotMass;
  return (jacobianWorldToContactPointInWorldFrame.template topRows<3>() - J_com);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 1> getNormalizedCentroidalMomentumRate(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                  const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                                  const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input) {
  const Eigen::Matrix<SCALAR_T, 3, 1> gravityVector(SCALAR_T(0.0), SCALAR_T(0.0), SCALAR_T(-9.81));
  Eigen::Matrix<SCALAR_T, 6, 1> centroidalMomentumRate;
  centroidalMomentumRate << info.robotMass * gravityVector, Eigen::Matrix<SCALAR_T, 3, 1>::Zero();

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto contactForceInWorldFrame = centroidal_model::getContactForces(input, i, info);
    const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
    centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    centroidalMomentumRate.template tail<3>().noalias() += positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame);
  }  // end of i loop

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto contactForceInWorldFrame = centroidal_model::getContactForces(input, i, info);
    const auto contactTorqueInWorldFrame = centroidal_model::getContactTorques(input, i, info);
    const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
    centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    centroidalMomentumRate.template tail<3>().noalias() +=
        positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame) + contactTorqueInWorldFrame;
  }  // end of i loop

  // normalize by the total mass
  centroidalMomentumRate /= info.robotMass;

  return centroidalMomentumRate;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// Explicit template instantiation
template void updateCentroidalDynamics<scalar_t>(PinocchioInterface&, const CentroidalModelInfo&, const vector_t&);
template void updateCentroidalDynamics<ad_scalar_t>(PinocchioInterfaceCppAd&, const CentroidalModelInfoCppAd&, const ad_vector_t&);

template void updateCentroidalDynamicsDerivatives<scalar_t>(PinocchioInterface&, const CentroidalModelInfo& info, const vector_t&,
                                                            const vector_t&);
template void updateCentroidalDynamicsDerivatives<ad_scalar_t>(PinocchioInterfaceCppAd&, const CentroidalModelInfoCppAd& info,
                                                               const ad_vector_t&, const ad_vector_t&);

template const Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>& getCentroidalMomentumMatrix<scalar_t>(const PinocchioInterface&);
template const Eigen::Matrix<ad_scalar_t, 6, Eigen::Dynamic>& getCentroidalMomentumMatrix<ad_scalar_t>(const PinocchioInterfaceCppAd&);

template Eigen::Matrix<scalar_t, 3, 1> getPositionComToContactPointInWorldFrame<scalar_t>(const PinocchioInterface&,
                                                                                          const CentroidalModelInfo&, size_t);
template Eigen::Matrix<ad_scalar_t, 3, 1> getPositionComToContactPointInWorldFrame<ad_scalar_t>(const PinocchioInterfaceCppAd&,
                                                                                                const CentroidalModelInfoCppAd&, size_t);

template Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame<scalar_t>(
    const PinocchioInterface&, const CentroidalModelInfo&, size_t);
template Eigen::Matrix<ad_scalar_t, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame<ad_scalar_t>(
    const PinocchioInterfaceCppAd&, const CentroidalModelInfoCppAd&, size_t);

template Eigen::Matrix<scalar_t, 6, 1> getNormalizedCentroidalMomentumRate<scalar_t>(const PinocchioInterface&, const CentroidalModelInfo&,
                                                                                     const vector_t&);
template Eigen::Matrix<ad_scalar_t, 6, 1> getNormalizedCentroidalMomentumRate<ad_scalar_t>(const PinocchioInterfaceCppAd&,
                                                                                           const CentroidalModelInfoCppAd&,
                                                                                           const ad_vector_t&);

}  // namespace ocs2
