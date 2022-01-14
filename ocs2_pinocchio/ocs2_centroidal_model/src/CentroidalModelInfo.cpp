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

std::string toString(CentroidalModelType type) {
  switch (type) {
    case CentroidalModelType::FullCentroidalDynamics:
      return "FullCentroidalDynamics";
    case CentroidalModelType::SingleRigidBodyDynamics:
      return "SingleRigidBodyDynamics";
    default:
      return "Undefined CentroidalModelType!";
  }
}

std::ostream& operator<<(std::ostream& os, CentroidalModelType type) {
  os << toString(type);
  return os;
}

template <>
template <>
CentroidalModelInfoCppAd CentroidalModelInfo::toCppAd() const {
  CentroidalModelInfoCppAd cppAdInfo;

  cppAdInfo.centroidalModelType = this->centroidalModelType;
  cppAdInfo.numThreeDofContacts = this->numThreeDofContacts;
  cppAdInfo.numSixDofContacts = this->numSixDofContacts;
  cppAdInfo.endEffectorFrameIndices = this->endEffectorFrameIndices;
  cppAdInfo.generalizedCoordinatesNum = this->generalizedCoordinatesNum;
  cppAdInfo.actuatedDofNum = this->actuatedDofNum;
  cppAdInfo.stateDim = this->stateDim;
  cppAdInfo.inputDim = this->inputDim;
  cppAdInfo.robotMass = ad_scalar_t(this->robotMass);
  cppAdInfo.qPinocchioNominal = this->qPinocchioNominal.cast<ad_scalar_t>();
  cppAdInfo.centroidalInertiaNominal = this->centroidalInertiaNominal.cast<ad_scalar_t>();
  cppAdInfo.comToBasePositionNominal = this->comToBasePositionNominal.cast<ad_scalar_t>();

  return cppAdInfo;
}

// explicit template instantiation
template struct ocs2::CentroidalModelInfoTpl<ocs2::scalar_t>;
template struct ocs2::CentroidalModelInfoTpl<ocs2::ad_scalar_t>;

}  // namespace ocs2
