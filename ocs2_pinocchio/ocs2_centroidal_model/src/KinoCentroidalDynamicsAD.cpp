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

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/container/aligned-vector.hpp>

#include <ocs2_centroidal_model/KinoCentroidalDynamicsAD.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
KinoCentroidalDynamicsAD::KinoCentroidalDynamicsAD(size_t stateDim, size_t inputDim,
                                                   const CentroidalModelPinocchioInterface<ad_scalar_t>& centroidalModelPinocchioInterface)
    : SystemDynamicsBaseAD(stateDim, inputDim), centroidalModelPinocchioInterface_(centroidalModelPinocchioInterface){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::ad_vector_t KinoCentroidalDynamicsAD::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                          const ad_vector_t& parameters) const {
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  assert(GENERALIZED_VEL_NUM == state.rows() - 6);
  auto& centroidalModelInfo = centroidalModelPinocchioInterface_.getCentroidalModelInfo();

  ad_vector_t stateDerivative = ad_vector_t::Zero(state.rows());

  centroidalModelPinocchioInterface_.updatePinocchioJointPositions(state);
  centroidalModelPinocchioInterface_.updatePinocchioJointVelocities(state, input);
  if (centroidalModelInfo.centroidalModelType ==
      CentroidalModelPinocchioInterface<ad_scalar_t>::CentroidalModelType::SingleRigidBodyDynamics) {
    pinocchio::forwardKinematics(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData(),
                                 centroidalModelInfo.qPinocchio);
  }
  pinocchio::updateFramePlacements(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData());

  // compute center of mass acceleration and derivative of the normalized angular momentum
  stateDerivative.head(6) = centroidalModelPinocchioInterface_.normalizedCentroidalMomentumRate(input);

  // derivatives of the floating base variables + joint velocities
  stateDerivative.tail(GENERALIZED_VEL_NUM) = centroidalModelInfo.vPinocchio;

  return stateDerivative;
}

}  // namespace ocs2