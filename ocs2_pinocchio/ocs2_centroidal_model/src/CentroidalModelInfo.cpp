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

#include "ocs2_centroidal_model/CentroidalModelInfo.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
CentroidalModelInfoTpl<SCALAR>::CentroidalModelInfoTpl(const PinocchioInterfaceTpl<SCALAR>& interface, const CentroidalModelType& type,
                                                       const vector_t& qNominal, const std::vector<std::string>& threeDofContactNames,
                                                       const std::vector<std::string>& sixDofContactNames) {
  const auto& model = interface.getModel();
  auto data = interface.getData();
  centroidalModelType = type;
  numThreeDofContacts = threeDofContactNames.size();
  numSixDofContacts = sixDofContactNames.size();
  generalizedCoordinatesNum = model.nq;
  actuatedDofNum = generalizedCoordinatesNum - 6;
  stateDim = generalizedCoordinatesNum + 6;
  inputDim = actuatedDofNum + 3 * numThreeDofContacts + 6 * numSixDofContacts;
  robotMass = pinocchio::computeTotalMass(model);
  for (const auto& name : threeDofContactNames) {
    endEffectorFrameIndices.push_back(model.getBodyId(name));
  }
  for (const auto& name : sixDofContactNames) {
    endEffectorFrameIndices.push_back(model.getBodyId(name));
  }

  // make sure the nominal base frame is aligned with the world frame
  qPinocchioNominal = qNominal;
  qPinocchioNominal.template head<6>().setZero();
  centroidalInertiaNominal.setZero();
  comToBasePositionNominal.setZero();
  if (centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    const vector_t vPinocchioNominal = vector_t::Zero(generalizedCoordinatesNum);
    pinocchio::ccrba(model, data, qPinocchioNominal, vPinocchioNominal);
    centroidalInertiaNominal = data.Ig.inertia().matrix();
    comToBasePositionNominal = qPinocchioNominal.template head<3>() - data.com[0];
  }
}

// explicit template instantiation
template class ocs2::CentroidalModelInfoTpl<ocs2::scalar_t>;
template class ocs2::CentroidalModelInfoTpl<ocs2::ad_scalar_t>;

}  // namespace ocs2
