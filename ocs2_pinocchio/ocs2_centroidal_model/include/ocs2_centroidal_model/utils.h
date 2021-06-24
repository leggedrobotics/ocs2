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

#pragma once

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace ocs2 {

/**
 * Get the inverse of the sub-block of the centroidal momentum matrix which corresponds to the floating base variables.
 *  Ab_inv = [  1/m I_{3,3},    -1/m*Ab_12*Ab_22^(-1),
 *                 O_{3,3},           Ab_22^(-1)     ]
 *
 * @param [in] A(q): centroidal momentum matrix
 * @return Ab_inv(q): inverse of the 6x6 left-block of A(q)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab) {
  const SCALAR_T mass = Ab(0, 0);
  Eigen::Matrix<SCALAR_T, 3, 3> Ab_22_inv = Ab.template block<3, 3>(3, 3).inverse();
  Eigen::Matrix<SCALAR_T, 6, 6> Ab_inv = Eigen::Matrix<SCALAR_T, 6, 6>::Zero();
  Ab_inv << 1.0 / mass * Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), -1.0 / mass * Ab.template block<3, 3>(0, 3) * Ab_22_inv,
      Eigen::Matrix<SCALAR_T, 3, 3>::Zero(), Ab_22_inv;
  return Ab_inv;
}

/**
 * Updates the centroidal momentum matrix in data.Ag and the CoM position in data.com[0]
 * for the FullCentroidalDynamics model and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 *
 * @remark: This function also internally calls pinocchio::forwardKinematics(model, data, q),
 * pinocchio::computeJointJacobians(model, data, q) (only for the FullCentroidalDynamics case),
 * and pinocchio::updateFramePlacements(model, data).
 */
template <typename SCALAR_T>
void updateCentroidalDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                              const Eigen::Matrix<SCALAR_T, -1, 1>& q) {
  const auto& model = interface.getModel();
  auto& data = interface.getData();
  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalMap(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      const Eigen::Matrix<SCALAR_T, 3, 1> eulerAnglesZyx = q.template segment<3>(3);
      const auto& mappingZyx = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAnglesZyx);
      const auto& rotationBaseToWorld = getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
      const Eigen::Matrix<SCALAR_T, 3, 1> comToBasePositionInWorld = rotationBaseToWorld * info.comToBasePositionNominal;
      const auto& skewSymmetricMap = skewSymmetricMatrix(comToBasePositionInWorld);
      const auto mat1 = rotationBaseToWorld * info.centroidalInertiaNominal;
      const auto mat2 = rotationBaseToWorld.transpose() * mappingZyx;
      Eigen::Matrix<SCALAR_T, 6, 6> Ab = Eigen::Matrix<SCALAR_T, 6, 6>::Zero();
      Ab.template topLeftCorner<3, 3>().diagonal().array() = info.robotMass;
      Ab.template topRightCorner<3, 3>() = info.robotMass * skewSymmetricMap * mappingZyx;
      Ab.template bottomRightCorner<3, 3>() = mat1 * mat2;
      Eigen::Matrix<SCALAR_T, -1, -1> A = Eigen::Matrix<SCALAR_T, -1, -1>::Zero(6, info.generalizedCoordinatesNum);
      A.template leftCols<6>() = Ab;
      data.Ag = A;
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

/**
 * Updates the centroidal momentum derivatives (such as in data.dHdq) for the FullCentroidalDynamics model
 * and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
 *
 * @remark: This function also internally calls pinocchio::forwardKinematics(model, data, q),
 * pinocchio::computeJointJacobians(model, data, q), and pinocchio::updateFramePlacements(model, data).
 */
template <typename SCALAR_T>
void updateCentroidalDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                         const Eigen::Matrix<SCALAR_T, -1, 1>& q, const Eigen::Matrix<SCALAR_T, -1, 1>& v) {
  const auto& model = interface.getModel();
  auto& data = interface.getData();
  Eigen::Matrix<SCALAR_T, -1, 1> a;
  Eigen::Matrix<SCALAR_T, 6, -1> dhdq;
  Eigen::Matrix<SCALAR_T, 6, -1> dhdotdq;
  Eigen::Matrix<SCALAR_T, 6, -1> dhdotdv;
  Eigen::Matrix<SCALAR_T, 6, -1> dhdotda;
  a.setZero(info.generalizedCoordinatesNum);
  dhdq.resize(6, info.generalizedCoordinatesNum);
  dhdotdq.resize(6, info.generalizedCoordinatesNum);
  dhdotdv.resize(6, info.generalizedCoordinatesNum);
  dhdotda.resize(6, info.generalizedCoordinatesNum);
  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalDynamicsDerivatives(model, data, q, v, a, dhdq, dhdotdq, dhdotdv, dhdotda);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      //      Eigen::Matrix<SCALAR_T, -1, 1> qSRBD = info.qPinocchioNominal;
      //      qSRBD.template head<6>() = q.template head<6>();
      //      Eigen::Matrix<SCALAR_T, -1, 1> vSRBD = Eigen::Matrix<SCALAR_T, -1, 1>::Zero(info.generalizedCoordinatesNum);
      //      vSRBD.template head<6>() = v.template head<6>();
      //      pinocchio::computeCentroidalDynamicsDerivatives(model, data, qSRBD, vSRBD, a, dhdq, dhdotdq, dhdotdv, dhdotda);
      const Eigen::Matrix<SCALAR_T, 3, 1> eulerAnglesZyx = q.template segment<3>(3);
      const Eigen::Matrix<SCALAR_T, 3, 1> eulerAnglesZyxDerivatives = v.template segment<3>(3);
      data.dHdq.setZero();
      data.dHdq.template block<6, 3>(0, 3) = getCentroidalMomentumZyxGradient(info, eulerAnglesZyx, eulerAnglesZyxDerivatives);
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
      break;
    }
  }
}

/**
 * Computes derivatives of the mapping (ZYX-Euler angles derivatives --> Global angular velocities)
 * with respect to the base orientation (ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the mapping w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> dTdz, dTdy, dTdx;
  dTdz << SCALAR_T(0),     -cos(z),        -cos(y)*sin(z),
          SCALAR_T(0),      -sin(z),        cos(y)*cos(z),
          SCALAR_T(0),     SCALAR_T(0),      SCALAR_T(0);

  dTdy << SCALAR_T(0),      SCALAR_T(0),        -sin(y)*cos(z),
          SCALAR_T(0),      SCALAR_T(0),        -sin(y)*sin(z),
          SCALAR_T(0),      SCALAR_T(0),          -cos(y);

  dTdx.setZero();
  // clang-format on

  return {dTdz, dTdy, dTdx};
}

/**
 * Computes derivatives of the rotation matrix (base frame --> world frame) with respect to
 * the base orientation (in ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the rotation matrix w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T dc1 = -s1;
  const SCALAR_T dc2 = -s2;
  const SCALAR_T dc3 = -s3;
  const SCALAR_T ds1 = c1;
  const SCALAR_T ds2 = c2;
  const SCALAR_T ds3 = c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> dRdz, dRdy, dRdx;
  dRdz << dc1 * c2,      dc1 * s2 * s3 - ds1 * c3,       dc1 * s2 * c3 + ds1 * s3,
          ds1 * c2,      ds1 * s2 * s3 + dc1 * c3,       ds1 * s2 * c3 - dc1 * s3,
          SCALAR_T(0),       SCALAR_T(0),                      SCALAR_T(0);

  dRdy << c1 * dc2,      c1 * ds2 * s3,       c1 * ds2 * c3,
          s1 * dc2,      s1 * ds2 * s3,       s1 * ds2 * c3,
           -ds2,            dc2 * s3,            dc2 * c3;

  dRdx << SCALAR_T(0),      c1 * s2 * ds3 - s1 * dc3,       c1 * s2 * dc3 + s1 * ds3,
          SCALAR_T(0),      s1 * s2 * ds3 + c1 * dc3,       s1 * s2 * dc3 - c1 * ds3,
          SCALAR_T(0),                c2 * ds3,                      c2 * dc3;
  // clang-format on

  return {dRdz, dRdy, dRdx};
}

/**
 * Computes derivatives of centroidal momentum with respect to the base orientation (in ZYX-Euler angles)
 *
 * @param [in] info: centroidal model information
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @param [in] eulerAnglesDerivatives: derivatives of ZYX-Euler angles extracted from vPinocchio
 * @return Derivative of centroidal momentum w.r.t the ZYX-Euler Angles
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 3> getCentroidalMomentumZyxGradient(const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                               const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                               const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesDerivatives) {
  const auto& T = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  const auto& R = getRotationMatrixFromZyxEulerAngles(eulerAngles);
  const Eigen::Matrix<SCALAR_T, 3, 1> r = R * info.comToBasePositionNominal;
  const auto& S = skewSymmetricMatrix(r);
  Eigen::Matrix<SCALAR_T, 6, 3> dhdq = Eigen::Matrix<SCALAR_T, 6, 3>::Zero();
  const auto& dT = getMappingZyxGradient(eulerAngles);
  const auto& dR = getRotationMatrixZyxGradient(eulerAngles);
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> dS;
  Eigen::Matrix<SCALAR_T, 3, 1> dr;
  for (size_t i = 0; i < 3; i++) {
    dr.noalias() = dR[i] * info.comToBasePositionNominal;
    dS[i] = skewSymmetricMatrix(dr);
  }

  const auto& m = info.robotMass;
  const auto& I = info.centroidalInertiaNominal;
  const auto& Rtranspose = R.transpose();
  for (size_t i = 0; i < 3; i++) {
    dhdq.template block<3, 1>(0, i).noalias() = m * (dS[i] * T + S * dT[i]) * eulerAnglesDerivatives;
    dhdq.template block<3, 1>(3, i).noalias() =
        (dR[i] * I * Rtranspose * T + R * I * dR[i].transpose() * T + R * I * Rtranspose * dT[i]) * eulerAnglesDerivatives;
  }

  return dhdq;
}
}  // namespace ocs2
