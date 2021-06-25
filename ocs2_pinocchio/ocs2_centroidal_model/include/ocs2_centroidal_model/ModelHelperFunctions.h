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

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelInfo.h"

namespace ocs2 {

/**
 * Get the inverse of the sub-block of the centroidal momentum matrix which corresponds to the floating base variables.
 *  Ab_inv = [  1/m I_{3,3},    -1/m*Ab_12*Ab_22^(-1),
 *              O_{3,3},         Ab_22^(-1)          ]
 *
 * @param [in] A(q): centroidal momentum matrix
 * @return Ab_inv(q): inverse of the 6x6 left-block of A(q)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab);

/**
 * Updates the centroidal momentum matrix in data.Ag and the CoM position in data.com[0]
 * for the FullCentroidalDynamics model and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 *
 * @remark: This function also internally calls:
 *       pinocchio::forwardKinematics(model, data, q)
 *       pinocchio::computeJointJacobians(model, data, q) (only for the FullCentroidalDynamics case)
 *       pinocchio::updateFramePlacements(model, data)
 */
template <typename SCALAR_T>
void updateCentroidalDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                              const Eigen::Matrix<SCALAR_T, -1, 1>& q);

/**
 * Updates the centroidal momentum derivatives (such as in data.dHdq) for the FullCentroidalDynamics model
 * and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
 *
 * @remark: This function also internally calls:
 *       pinocchio::forwardKinematics(model, data, q)
 *       pinocchio::computeJointJacobians(model, data, q)
 *       pinocchio::updateFramePlacements(model, data)
 */
template <typename SCALAR_T>
void updateCentroidalDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                         const Eigen::Matrix<SCALAR_T, -1, 1>& q, const Eigen::Matrix<SCALAR_T, -1, 1>& v);

/**
 * Computes derivatives of the mapping (ZYX-Euler angles derivatives --> Global angular velocities)
 * with respect to the base orientation (ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the mapping w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

/**
 * Computes derivatives of the rotation matrix (base frame --> world frame) with respect to
 * the base orientation (in ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the rotation matrix w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

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
                                                               const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesDerivatives);

}  // namespace ocs2

#include "implementation/ModelHelperFunctionsImpl.h"
